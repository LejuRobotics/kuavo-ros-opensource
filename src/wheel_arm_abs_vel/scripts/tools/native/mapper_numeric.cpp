// Hand-only adaptation of mapper_fused_kernels_c5369d64.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
namespace py=pybind11;
using M3=Eigen::Matrix3d;using M6=Eigen::Matrix<double,6,6>;using V3=Eigen::Vector3d;using V6=Eigen::Matrix<double,6,1>;
M3 rotation_fn(const V3& v,bool jacobian){
 double x=v[0],y=v[1],z=v[2],xx=x*x,yy=y*y,zz=z*z,a2=xx+yy+zz,a=std::sqrt(a2),s,c;
 if(jacobian){if(a<1e-6){s=.5-a2/24;c=1./6-a2/120;}else{s=(1-std::cos(a))/a2;c=(a-std::sin(a))/(a2*a);}}
 else {if(a<1e-6){s=1-a2/6;c=.5-a2/24;}else{s=std::sin(a)/a;c=(1-std::cos(a))/a2;}}
 M3 out;out<<1-c*(yy+zz),c*x*y-s*z,c*x*z+s*y,c*x*y+s*z,1-c*(xx+zz),c*y*z-s*x,c*x*z-s*y,c*y*z+s*x,1-c*(xx+yy);return out;
}
V3 rotation_log(const M3& R) {
 Eigen::Quaterniond q(R); q.normalize();
 if(q.w()<0.)q.coeffs()*=-1.;
 const double n=q.vec().norm();
 if(n<1.e-8)return (2.+n*n/3.)*q.vec();
 return (2.*std::atan2(n,q.w())/n)*q.vec();
}
py::tuple rotation_update(const M3& measured,const M3& R,const V3& w,const M6& old,double dt,double r,double q,double gain){
 M3 I=M3::Identity(),E=rotation_fn(w*dt,false),J=rotation_fn(w*dt,true)*dt;
 M6 Q=M6::Zero();const double nodes[]={.1127016653792583,.5,.8872983346207417};const double weights[]={5./18,4./9,5./18};
 for(int i=0;i<3;i++){double s=dt*nodes[i],scale=q*dt*weights[i];M3 B=rotation_fn(w*s,true)*s;
  Q.topLeftCorner<3,3>()+=scale*(B*B.transpose());Q.topRightCorner<3,3>()+=scale*B;
  Q.bottomLeftCorner<3,3>()+=scale*B.transpose();Q.bottomRightCorner<3,3>()+=scale*I;}
 M3 a00=E*old.topLeftCorner<3,3>()+J*old.bottomLeftCorner<3,3>();
 M3 a01=E*old.topRightCorner<3,3>()+J*old.bottomRightCorner<3,3>();
 M6 P;P.topLeftCorner<3,3>()=a00*E.transpose()+a01*J.transpose()+Q.topLeftCorner<3,3>();
 P.topRightCorner<3,3>()=a01+Q.topRightCorner<3,3>();
 P.bottomLeftCorner<3,3>()=old.bottomLeftCorner<3,3>()*E.transpose()+old.bottomRightCorner<3,3>()*J.transpose()+Q.bottomLeftCorner<3,3>();
 P.bottomRightCorner<3,3>()=old.bottomRightCorner<3,3>()+Q.bottomRightCorner<3,3>();
 M3 predicted_R=E*R;
 V3 residual=rotation_log(measured*predicted_R.transpose());
 Eigen::Matrix<double,6,3> K=(P.topLeftCorner<3,3>()+r*I).partialPivLu().solve(P.topRows<3>()).transpose();K.bottomRows<3>()*=gain;
 V6 correction=K*residual;M3 corrected_R=rotation_fn(correction.head<3>(),false)*predicted_R;V3 new_w=w+correction.tail<3>();
 M3 U=I-K.topRows<3>(),V=-K.bottomRows<3>();
 M3 b00=U*P.topLeftCorner<3,3>(),b01=U*P.topRightCorner<3,3>();
 M3 b10=V*P.topLeftCorner<3,3>()+P.bottomLeftCorner<3,3>(),b11=V*P.topRightCorner<3,3>()+P.bottomRightCorner<3,3>();
 M6 C;C.topLeftCorner<3,3>()=b00*U.transpose()+r*(K.topRows<3>()*K.topRows<3>().transpose());
 C.topRightCorner<3,3>()=b00*V.transpose()+b01+r*(K.topRows<3>()*K.bottomRows<3>().transpose());
 C.bottomLeftCorner<3,3>()=b10*U.transpose()+r*(K.bottomRows<3>()*K.topRows<3>().transpose());
 C.bottomRightCorner<3,3>()=b10*V.transpose()+b11+r*(K.bottomRows<3>()*K.bottomRows<3>().transpose());
 M3 reset=rotation_fn(correction.head<3>(),true);M6 result;
 result.topLeftCorner<3,3>()=reset*C.topLeftCorner<3,3>()*reset.transpose();result.topRightCorner<3,3>()=reset*C.topRightCorner<3,3>();
 result.bottomLeftCorner<3,3>()=C.bottomLeftCorner<3,3>()*reset.transpose();result.bottomRightCorner<3,3>()=C.bottomRightCorner<3,3>();
 M6 symmetric=.5*(result+result.transpose()).eval();
 return py::make_tuple(corrected_R,new_w,symmetric);
}
py::tuple position_update(py::array_t<double,py::array::c_style|py::array::forcecast> z,py::array_t<double,py::array::c_style|py::array::forcecast> x,const Eigen::Matrix2d& P,double dt,double r,double q,double gain){
 double p00=P(0,0),p01=P(0,1),p10=P(1,0),p11=P(1,1),dt2=dt*dt;
 double a=p00+dt*(p01+p10)+dt2*p11+q*dt2*dt/3.,b=p01+dt*p11+q*dt2/2.,c=p10+dt*p11+q*dt2/2.,d=p11+q*dt;
 double k0=a/(a+r),k1=c/(a+r)*gain,u=1.-k0,off=u*(.5*(b+c)-k1*a)+r*k0*k1;
 Eigen::Matrix2d result;result<<u*u*a+r*k0*k0,off,off,d-k1*(b+c)+k1*k1*(a+r);
 py::array_t<double> state({2,3});const double* obs=z.data();const double* prev=x.data();double* out=state.mutable_data();
 for(int i=0;i<3;i++){double predicted=prev[i]+dt*prev[i+3],residual=obs[i]-predicted;out[i]=predicted+k0*residual;out[i+3]=prev[i+3]+k1*residual;}
 return py::make_tuple(state,result);
}

py::object retarget(const V3& sr,const V3& er,const V3& hr,const M3& remove,const V3& anchor,double upper,double lower,const M3& source,const Eigen::Matrix4d& tool){
 V3 ud=er-sr,fd=hr-er;double un=ud.norm(),fn=fd.norm();
 if(un<1e-6||fn<1e-6)return py::none();
 ud/=un;fd/=fn;
 V3 hint=anchor+upper*ud,mapped=hint+lower*fd;
 M3 mappedR=remove*source,tcpR=mappedR*tool.topLeftCorner<3,3>();
 V3 tcp=mapped+mappedR*tool.topRightCorner<3,1>();
 return py::make_tuple(tcp,tcpR,hint,mapped,mappedR,ud,fd);
}
PYBIND11_MODULE(_mapper_numeric_hands,m){m.attr("api_version")=2;m.def("rotation_log",&rotation_log);m.def("rotation_exp",[](const V3& v){return rotation_fn(v,false);});m.def("position_update",&position_update);m.def("rotation_update",&rotation_update);m.def("retarget",&retarget);}
