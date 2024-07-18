#include <OpenSoT/utils/cartesian_utils.h>
#include <vector>
#include <wolf_controller/utils.h>

std::vector<float> _vertx;
std::vector<float> _verty;
float _testx;
float _testy;
int _res;

int main(int argc, char **argv)
{
    _vertx.resize(N_LEGS);
    _verty.resize(N_LEGS);

    _vertx[0] = 0.5;
    _verty[0] = 0.5;

    _vertx[1] = 0.5;
    _verty[1] = -0.5;

    _vertx[2] = -0.5;
    _verty[2] = -0.5;

    _vertx[3] = -0.5;
    _verty[3] = 0.5;

    // Inside point
    _testx = 0.0;
    _testy = 0.0;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    // Outside point
    _testx = 1.0;
    _testy = 1.0;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    // Vertex point
    _testx = 0.5;
    _testy = 0.5;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    // Inside point
    _testx = 0.45;
    _testy = 0.49;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    // Outside
    _vertx[0] = 0.349556;
    _verty[0] = 0.119084;
    _vertx[1] = -0.0712354;
    _verty[1] = 0.368615  ;
    _vertx[2] =  0.0929288;
    _verty[2] =  -0.410112;
    _vertx[3] = -0.376475 ;
    _verty[3] = -0.0700615;
    _testx = 0.00766264;
    _testy = -0.00556975;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    // Inside
    _vertx[0] = 0.349533;
    _verty[0] = 0.119114;
    _vertx[1] = -0.0712589;
    _verty[1] = 0.368655  ;
    _vertx[2] =  0.0928883;
    _verty[2] =  -0.410073;
    _vertx[3] = -0.376503;
    _verty[3] = -0.0700183;
    _testx = 0.00712562;
    _testy = -0.00508121;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;


    // Outside but should be inside
    _vertx[0] = 0.232579 ;
    _verty[0] = 0.368522 ;
    _vertx[1] = -0.347384;
    _verty[1] = 0.201729 ;
    _vertx[2] = -0.166703;
    _verty[2] = -0.323422;
    _vertx[3] = 0.310892 ;
    _verty[3] = -0.260148;

    //_vertx[0] = 0.310892 ;
    //_verty[0] = -0.260148;
    //_vertx[1] = 0.232579 ;
    //_verty[1] = 0.368522 ;
    //_vertx[2] = -0.166703;
    //_verty[2] = -0.323422;
    //_vertx[3] = -0.347384;
    //_verty[3] = 0.201729 ;
    _testx = 0.0147119;
    _testy = 0.00814953;

    _res = cartesian_utils::pnpoly(N_LEGS,_vertx.data(),_verty.data(),_testx,_testy);
    std::cout << _res << std::endl;
    if(_res==0)
      std::cout << "Outside! " <<std::endl;
    else
      std::cout << "Inside! " <<std::endl;

    return 0;
}
