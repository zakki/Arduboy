
#define NSUBSAMPLES  2
#define NAO_SAMPLES  4

struct Vec {
  float x;
  float y;
  float z;
};


struct Isect
{
  float t;
  Vec    p;
  Vec    n;
  int    hit;
};

struct Sphere
{
  Vec    center;
  float radius;

};

struct Plane
{
  Vec    p;
  Vec    n;

};

struct Ray
{
  Vec    org;
  Vec    dir;
};
