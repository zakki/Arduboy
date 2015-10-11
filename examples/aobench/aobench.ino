/*
 *
 *  aobench is originally written by Syoyo Fujita.
 */

#include <SPI.h>
#include <EEPROM.h>
#include "Arduboy.h"
#include "aobench.h"

Arduboy display;

unsigned long lTime;


Sphere spheres[3];
Plane  plane;

int cur_x;
//float fimg[WIDTH * HEIGHT];
float imgerror[HEIGHT];

static float vdot(Vec v0, Vec v1)
{
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

static void vcross(Vec *c, Vec v0, Vec v1)
{

  c->x = v0.y * v1.z - v0.z * v1.y;
  c->y = v0.z * v1.x - v0.x * v1.z;
  c->z = v0.x * v1.y - v0.y * v1.x;
}

static void vnormalize(Vec *c)
{
  float length = sqrt(vdot((*c), (*c)));

  if (fabs(length) > 1.0e-17) {
    c->x /= length;
    c->y /= length;
    c->z /= length;
  }
}

void
ray_sphere_intersect(Isect *isect, const Ray *ray, const Sphere *sphere)
{
  Vec rs;

  rs.x = ray->org.x - sphere->center.x;
  rs.y = ray->org.y - sphere->center.y;
  rs.z = ray->org.z - sphere->center.z;

  float B = vdot(rs, ray->dir);
  float C = vdot(rs, rs) - sphere->radius * sphere->radius;
  float D = B * B - C;

  if (D > 0.0) {
    float t = -B - sqrt(D);

    if ((t > 0.0) && (t < isect->t)) {
      isect->t = t;
      isect->hit = 1;

      isect->p.x = ray->org.x + ray->dir.x * t;
      isect->p.y = ray->org.y + ray->dir.y * t;
      isect->p.z = ray->org.z + ray->dir.z * t;

      isect->n.x = isect->p.x - sphere->center.x;
      isect->n.y = isect->p.y - sphere->center.y;
      isect->n.z = isect->p.z - sphere->center.z;

      vnormalize(&(isect->n));
    }
  }
}

void
ray_plane_intersect(Isect *isect, const Ray *ray, const Plane *plane)
{
  float d = -vdot(plane->p, plane->n);
  float v = vdot(ray->dir, plane->n);

  if (fabs(v) < 1.0e-17) return;

  float t = -(vdot(ray->org, plane->n) + d) / v;

  if ((t > 0.0) && (t < isect->t)) {
    isect->t = t;
    isect->hit = 1;

    isect->p.x = ray->org.x + ray->dir.x * t;
    isect->p.y = ray->org.y + ray->dir.y * t;
    isect->p.z = ray->org.z + ray->dir.z * t;

    isect->n = plane->n;
  }
}

void
orthoBasis(Vec *basis, Vec n)
{
  basis[2] = n;
  basis[1].x = 0.0; basis[1].y = 0.0; basis[1].z = 0.0;

  if ((n.x < 0.6) && (n.x > -0.6)) {
    basis[1].x = 1.0;
  } else if ((n.y < 0.6) && (n.y > -0.6)) {
    basis[1].y = 1.0;
  } else if ((n.z < 0.6) && (n.z > -0.6)) {
    basis[1].z = 1.0;
  } else {
    basis[1].x = 1.0;
  }

  vcross(&basis[0], basis[1], basis[2]);
  vnormalize(&basis[0]);

  vcross(&basis[1], basis[2], basis[0]);
  vnormalize(&basis[1]);
}

float drand48() {
  return random(10000) / 10000.0;
}

void ambient_occlusion(Vec *col, const Isect *isect)
{
  int    i, j;
  int    ntheta = NAO_SAMPLES;
  int    nphi   = NAO_SAMPLES;
  float eps = 0.0001;

  Vec p;

  p.x = isect->p.x + eps * isect->n.x;
  p.y = isect->p.y + eps * isect->n.y;
  p.z = isect->p.z + eps * isect->n.z;

  Vec basis[3];
  orthoBasis(basis, isect->n);

  float occlusion = 0.0;

  for (j = 0; j < ntheta; j++) {
    for (i = 0; i < nphi; i++) {
      float theta = sqrt(drand48());
      float phi   = 2.0 * M_PI * drand48();

      float x = cos(phi) * theta;
      float y = sin(phi) * theta;
      float z = sqrt(1.0 - theta * theta);

      // local -> global
      float rx = x * basis[0].x + y * basis[1].x + z * basis[2].x;
      float ry = x * basis[0].y + y * basis[1].y + z * basis[2].y;
      float rz = x * basis[0].z + y * basis[1].z + z * basis[2].z;

      Ray ray;

      ray.org = p;
      ray.dir.x = rx;
      ray.dir.y = ry;
      ray.dir.z = rz;

      Isect occIsect;
      occIsect.t   = 1.0e+17;
      occIsect.hit = 0;

      ray_sphere_intersect(&occIsect, &ray, &spheres[0]);
      ray_sphere_intersect(&occIsect, &ray, &spheres[1]);
      ray_sphere_intersect(&occIsect, &ray, &spheres[2]);
      ray_plane_intersect (&occIsect, &ray, &plane);

      if (occIsect.hit) occlusion += 1.0;

    }
  }

  occlusion = (ntheta * nphi - occlusion) / (float)(ntheta * nphi);

  col->x = occlusion;
  col->y = occlusion;
  col->z = occlusion;
}

unsigned char
clamp(float f)
{
  int i = (int)(f * 255.5);

  if (i < 0) i = 0;
  if (i > 255) i = 255;

  return (unsigned char)i;
}

void
init_scene()
{
  spheres[0].center.x = -2.0;
  spheres[0].center.y =  0.0;
  spheres[0].center.z = -3.5;
  spheres[0].radius = 0.5;

  spheres[1].center.x = -0.5;
  spheres[1].center.y =  0.0;
  spheres[1].center.z = -3.0;
  spheres[1].radius = 0.5;

  spheres[2].center.x =  1.0;
  spheres[2].center.y =  0.0;
  spheres[2].center.z = -2.2;
  spheres[2].radius = 0.5;

  plane.p.x = 0.0;
  plane.p.y = -0.5;
  plane.p.z = 0.0;

  plane.n.x = 0.0;
  plane.n.y = 1.0;
  plane.n.z = 0.0;

}

void setup() {
  display.start();
  init_scene();
  for (int i = 0; i < HEIGHT; i++)
    imgerror[i] = 0;
  display.display();
  display.setCursor(24, 22);
  display.setTextSize(2);
  display.print("AOBENCH");
  display.setTextSize(1);
  display.setCursor(18, 55);
  display.print("Press Any Button");
  display.display();

  while (!display.getInput());

  delay(500);
  lTime = millis();
  cur_x = 0;
  display.clearDisplay();
}

void loop() {
  int x, y;
  int u, v;

  x = cur_x;
  float scale = min(1.0 / (WIDTH / 2.0), 1.0 / (HEIGHT / 2.0));
  {
    for (y = 0; y < HEIGHT; y++) {
      display.drawPixel(x, y, WHITE);
      display.display();
      float fimg = 0;

      for (v = 0; v < NSUBSAMPLES; v++) {
        for (u = 0; u < NSUBSAMPLES; u++) {
          float px = (x + (u / (float)NSUBSAMPLES) - (WIDTH / 2.0)) * scale;
          float py = -(y + (v / (float)NSUBSAMPLES) - (HEIGHT / 2.0)) * scale;

          Ray ray;

          ray.org.x = 0.0;
          ray.org.y = 0.0;
          ray.org.z = 0.0;

          ray.dir.x = px;
          ray.dir.y = py;
          ray.dir.z = -1.0;
          vnormalize(&(ray.dir));

          Isect isect;
          isect.t   = 1.0e+17;
          isect.hit = 0;

          ray_sphere_intersect(&isect, &ray, &spheres[0]);
          ray_sphere_intersect(&isect, &ray, &spheres[1]);
          ray_sphere_intersect(&isect, &ray, &spheres[2]);
          ray_plane_intersect (&isect, &ray, &plane);

          if (isect.hit) {
            Vec col;
            ambient_occlusion(&col, &isect);

            fimg += col.x;
          }
        }
      }
      fimg /= (float)(NSUBSAMPLES * NSUBSAMPLES);

      float val;
      float offset = imgerror[y];
      if (offset + fimg < 0.5) {
        display.drawPixel(x, y, BLACK);
        val = 0;
      } else {
        display.drawPixel(x, y, WHITE);
        val = 1;
      }
      float err = (fimg - val) / 16;
      imgerror[y] = err * 6;
      if (y > 0)
        imgerror[y - 1] += err * 3;
      if (y + 1 < HEIGHT)
        imgerror[y + 1] += err * 7;
      display.fillRect(64, 0, 127, 8, BLACK); // Box border
      display.setCursor(64, 0);
      display.print(fimg);
      display.display();
    }
  }

  cur_x++;
  if (cur_x >= WIDTH) {
    cur_x = 0;

    display.clearDisplay();
    lTime = millis();
  } else {
    unsigned long now = millis();
    unsigned long t = (now - lTime) / 1000;
    display.fillRect(0, 0, 64, 8, BLACK); // Box border
    display.setCursor(0, 0);
    display.print(t);
    display.print("sec");
    display.display();
  }
}
