#include "Vec.h"
#include <cmath>

using std::atan2;
using std::sqrt;

#define CONSTWURZELDREIHALBE 0.86602540378443859659
#define CONSTWURZELZWEIHALBE 0.70710678118654757274

const Vec Vec::unit_vector_x (1,0);
const Vec Vec::unit_vector_y (0,1);
const Vec Vec::zero_vector (0,0);

std::ostream& operator<< (std::ostream& os, const Vec& v) {
  os <<' '<< v.x << ' ' << v.y<<' ';
  return os;
}

bool Vec::operator== (const Vec v) const throw () {
  return (x==v.x) && (y==v.y);
}

bool Vec::operator!= (const Vec v) const throw () {
  return (x!=v.x) || (y!=v.y);
}

Vec Vec::operator+ (const Vec v) const throw () {
  Vec res (*this);
  res+=v;
  return res;
}

const Vec& Vec::operator+= (const Vec v) throw () {
  x+=v.x;
  y+=v.y;
  return *this;
}

Vec Vec::operator- (const Vec v) const throw () {
  Vec res (*this);
  res-=v;
  return res;
}

const Vec& Vec::operator-= (const Vec v) throw () {
  x-=v.x;
  y-=v.y;
  return *this;
}

Vec Vec::operator- () const throw () {
  return Vec (-x, -y);
}

const Vec& Vec::operator*= (double s) throw () {
  x*=s;
  y*=s;
  return *this;
}

const Vec& Vec::operator/= (double s) throw () {
  x/=s;
  y/=s;
  return *this;
}

Vec operator* (Vec v, double s) throw () {
  Vec res (v);
  res*=s;
  return res;
}

Vec operator* (double s, Vec v) throw () {
  Vec res (v);
  res*=s;
  return res;
}

Vec operator/ (Vec v, double s) throw () {
  Vec res(v);
  res/=s;
  return res;
}

double Vec::operator* (const Vec v) const throw () {
  return x*v.x+y*v.y;
}

Vec Vec::rotate_twelvth () const throw () {
  return Vec (x*CONSTWURZELDREIHALBE-y*0.5, x*0.5+y*CONSTWURZELDREIHALBE);
}
Vec Vec::rotate_eleven_twelvth () const throw () {
  return Vec (x*CONSTWURZELDREIHALBE+y*0.5, -0.5*x+y*CONSTWURZELDREIHALBE);
}
Vec Vec::rotate_eighth () const throw () {
  return Vec (CONSTWURZELZWEIHALBE*(x-y), CONSTWURZELZWEIHALBE*(x+y));
}
Vec Vec::rotate_seven_eighth () const throw () {
  return Vec (CONSTWURZELZWEIHALBE*(x+y), CONSTWURZELZWEIHALBE*(y-x));
}
Vec Vec::rotate_sixth () const throw () {
  return Vec (0.5*x-y*CONSTWURZELDREIHALBE, x*CONSTWURZELDREIHALBE+y*0.5);
}
Vec Vec::rotate_five_sixth () const throw () {
  return Vec (0.5*x+y*CONSTWURZELDREIHALBE, -x*CONSTWURZELDREIHALBE+y*0.5);
}
Vec Vec::rotate_quarter () const throw () {
  return Vec (-y,x);
}
Vec Vec::rotate_three_quarters () const throw () {
  return Vec (y,-x);
}
Vec Vec::rotate_half () const throw () {
  return Vec (-x,-y);
}

Vec Vec::mirror (const Vec v) const throw () {
  double n=v.x*v.x+v.y*v.y;
  n = (n <= 0 ? 1 : n);   // fuer den Fall, in dem v==0 ist --> Punktspiegelung
  double f =2*(v.x*x+v.y*y)/n;
  return Vec (f*v.x-x, f*v.y-y);
}
Vec Vec::mirror_x () const throw () {
  return Vec (x,-y);
}
Vec Vec::mirror_y () const throw () {
  return Vec (-x,y);
}
Vec Vec::mirror_eighth () const throw () {
  return Vec (y,x);
}
Vec Vec::mirror_three_eighth () const throw () {
  return Vec (-y,-x);
}
 
double Vec::squared_length () const throw () {
  return x*x+y*y;
}

double Vec::length () const throw () {
  return sqrt(x*x+y*y);
}
	
Vec Vec::setLength( double len) const throw()
{
	if(length() > 0.0001)
		return ( (*this) * (len/length()) );
	else
		return (*this);
}

Angle Vec::angle () const throw () {
  return atan2 (y,x);
}

Angle Vec::angle (const Vec v) const throw () {
  double z = (x*v.x+y*v.y)/sqrt((x*x+y*y)*(v.x*v.x+v.y*v.y));  // normiertes Skalarprodukt
  double z1=1-(z*z);
  if (z1<0) z1=0;  // Wegen Ungenauikeit kann z*z gr�sser 1 sein.
  double phi = atan2 (sqrt(z1),z);  // Pythagoras
  return Angle ( -y*v.x+x*v.y >= 0 ? phi : -phi);  // Skalarprodukt mit Orthogonalvektor
}

bool linearly_independent (const Vec v1, const Vec v2) {
  return (v1.x*v2.y-v1.y*v2.x!=0);
}

Vec Vec::normalize () const throw () {
  double len = length();
  if (len==0)
    return Vec (0,0);
  return (1.0/len)*(*this);
}
