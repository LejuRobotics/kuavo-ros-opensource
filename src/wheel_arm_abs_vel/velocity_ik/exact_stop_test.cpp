#include "exact_stop.h"
#include <iostream>
#include <stdexcept>
namespace { void check(bool ok,const char* what){ if(!ok) throw std::runtime_error(what);} }
int main(){
 // A bisection that only evaluates an interior predicate loses an endpoint the
 // ORIGINAL stopping predicate explicitly certifies. Seed it exactly.
 auto predicate=[](double u){return u<=0.;};
 double left=-.0801,right=.0799;
 for(int k=0;k<40;++k){const double mid=.5*(left+right);if(predicate(mid))left=mid;else right=mid;}
 check(left<0.&&predicate(0.),"legacy bisection loses accepted zero");
 left=velocity_ik::certifiedUpperZeroSeed(-.0801,.0799,predicate(0.));right=.0799;
 for(int k=0;k<40;++k){const double mid=.5*(left+right);if(predicate(mid))left=mid;else right=mid;}
 check(left==0.,"certified upper zero endpoint retained exactly");
 check(velocity_ik::certifiedLowerZeroSeed(-.0799,.0801,true)==0.,"negative-side symmetry");
 check(velocity_ik::certifiedUpperZeroSeed(-.08,-.01,true)==-.08,"no zero outside bracket");
 check(velocity_ik::certifiedLowerZeroSeed(.01,.08,true)==.08,"no zero outside positive bracket");
 check(velocity_ik::certifiedUpperZeroSeed(-.08,.08,false)==-.08,"no zero without original certificate");
 check(velocity_ik::certifiedLowerZeroSeed(-.08,.08,false)==.08,"no zero without original certificate (lower)");
 std::cout<<"PASS certified zero-endpoint seeding\n";
 return 0;
}
