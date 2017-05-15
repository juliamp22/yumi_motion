
#include <mt/mt.h>

void printTransform(mt::Transform T)
{
    mt::Rotation R=T.getRotation();
    mt::Matrix3x3 M=R.getMatrix();
    mt::Point3 d = T.getTranslation();
    std::cout<<M[0][0]<<" "<<M[0][1]<<" "<<M[0][2]<<" "<<d[0]<< std::endl;
    std::cout<<M[1][0]<<" "<<M[1][1]<<" "<<M[1][2]<<" "<<d[1]<< std::endl;
    std::cout<<M[2][0]<<" "<<M[2][1]<<" "<<M[2][2]<<" "<<d[2]<< std::endl;
    std::cout<<"0 0 0 1"<< std::endl;
    std::cout<<"T = (" << d[0] << ", " << d[1] <<", " << d[2];
    std::cout<< ", "<< R[0] << ", " << R[1] << ", "<< R[2] << ", "<< R[3] <<")"<<std::endl;
}


int main()
{
     std::cout<<"Simple demo of the use of transforms with the mt library\n"<<std::endl;
     //--------------------------------------------
     mt::Rotation R1(mt::Matrix3x3(1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0));
     mt::Point3 d1(1.0, 2.0, 3.0);
     mt::Transform T1(R1, d1);
     //--------------------------------------------
     mt::Transform T2;
     T2.setRotation(mt::Matrix3x3(0.0, 1.0, 0.0,
                                  1.0, 0.0, 0.0,
                                  0.0, 0.0, -1.0));
     T2.setTranslation(mt::Point3(0.0, 1.0, 2.0));
     //--------------------------------------------
     mt::Transform invT2(T2.inverse());
     //--------------------------------------------
     mt::Transform T3;
     T3=T2*T2.inverse();
     //--------------------------------------------
     mt::Transform T4;
     T4=T1*T2.inverse();


     //--------------------------------------------
     std::cout<<"\nT1:" <<std::endl;
     printTransform(T1);
     std::cout<<"\nT2:" <<std::endl;
     printTransform(T2);
     std::cout<<"\ninvT2:" <<std::endl;
     printTransform(invT2);
     std::cout<<"\nT3:" <<std::endl;
     printTransform(T3);
     std::cout<<"\nT4:" <<std::endl;
     printTransform(T4);
}
