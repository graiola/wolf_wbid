#include <gtest/gtest.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/Logger.hpp>
#include <thread>
#include <functional>

using XBot::Logger;


namespace {

class testUtils: public ::testing::Test {
    
public:
    
    
    void test_mt_log_func(int th_id) {
        for(int i = 0; i < 5000; i++){
            Logger::info() << "Thread #" << th_id << " printing to logger!" << Logger::endl();
            Logger::error("Thread #%d printing to logger", th_id);
            usleep(1000);
        }
     }

protected:

     testUtils(){
         
         
     }

     virtual ~testUtils() {
     }

     virtual void SetUp() {
         
     }

     virtual void TearDown() {
     }
     
     
     
};


TEST_F(testUtils, checkFilterStepResponse)
{
    auto logger = XBot::MatLogger::getLogger("/tmp/checkFilterStepResponse");
    XBot::Utils::SecondOrderFilter<double> filter;
    
    double ts = 0.001;
    double eps = 1.0;
    
    int N = 15;
    double f_min = 0.1;
    double f_max = 1000;
    double k = std::pow(f_max - f_min, 1.0/N);
    std::vector<double> v_omega = {f_min * 2.0 * M_PI};
    for(int i = 1; i < N; i++)
    {
        v_omega.push_back(v_omega.back()*k);
    }
    
    filter.setOmega(v_omega[0]);
    filter.setTimeStep(ts);
    filter.setDamping(eps);
    
    Eigen::MatrixXd log(N, 20000);
    int n = 0;
    for(double omega : v_omega)
    {
        
        filter.setOmega(omega);
        filter.reset(0.0);
        
        logger->add("cutoff_freq", omega / (2*M_PI));
        
        for(int k = 0; k < 20000; k++)
        {
            double y_1 = filter.getOutput();
            log(n, k) = filter.process(1.0);
            double y_2 = filter.getOutput();
            
            ASSERT_TRUE( log(n,k) == y_2 );
            EXPECT_TRUE( (y_2 + 1e-12) >= y_1 );
        }
        
        n++;
    }
    
    logger->log("filter_output", log);
    
    logger->flush();
    
}

TEST_F (testUtils, checkFilterDcGainDouble) { 
    
    XBot::Utils::SecondOrderFilter<double> filter;
    
    double ts = 0.01;
    double cutoff = 10;
    double omega = 2*3.1415*cutoff; // 10 Hz cutoff
    double eps = 1.0;
    
    filter.setOmega(omega);
    filter.setTimeStep(ts);
    filter.setDamping(eps);
    
    filter.reset(0.0);
    
    double time = 0;
    
    double input = 1.0;
    double output;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_EQ( output, filter.getOutput());
        time += ts;
    }
    
    EXPECT_NEAR(filter.getOutput(), 1.0, 1e-3);
    
    
    filter.reset(2.0);
    time = 0;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_EQ( output, filter.getOutput());
        time += ts;
    }
    
    EXPECT_NEAR(filter.getOutput(), 1.0, 1e-3);
    
}

TEST_F(testUtils, testSecondOrderFilterArray)
{
    int channels = 6;
    XBot::Utils::SecondOrderFilterArray<double> _filters(channels);

    double ts = 0.01;
    double cutoff = 10;
    double omega = 2*3.1415*cutoff; // 10 Hz cutoff
    double eps = 1.0;

    for(unsigned int i = 0; i < channels; ++i)
    {
        EXPECT_TRUE(_filters.setDamping(eps, i));
        EXPECT_TRUE(_filters.setTimeStep(ts, i));
        EXPECT_TRUE(_filters.setOmega(omega, i));
    }

    for(unsigned int i = 0; i < channels; ++i)
    {
        EXPECT_DOUBLE_EQ(_filters.getDamping(i), eps);
        EXPECT_DOUBLE_EQ(_filters.getTimeStep(i), ts);
        EXPECT_DOUBLE_EQ(_filters.getOmega(i), omega);
    }

    double time = 0;

    std::vector<double> input, output;
    for(unsigned int i = 0; i < channels; ++i){
        input.push_back(1.);}

    double init = 2.;
    _filters.reset(init);
    //std::cout<<"filter init:"<<std::endl;
    for(unsigned int i = 0; i < channels; ++i){
        EXPECT_NEAR(_filters.getOutput()[i], 2.0, 1e-3);
        //std::cout<<_filters.getOutput()[i]<<std::endl;
    }

    while(time < 1.0/cutoff*10){
        output = _filters.process(input);
        for(unsigned int i = 0; i < channels; ++i)
            ASSERT_DOUBLE_EQ( output[i], _filters.getOutput()[i]);
        time += ts;
    }

    //std::cout<<"filter end:"<<std::endl;
    for(unsigned int i = 0; i < channels; ++i){
        EXPECT_NEAR(_filters.getOutput()[i], 1.0, 1e-3);
        //std::cout<<_filters.getOutput()[i]<<std::endl;
    }


}
 
 
TEST_F (testUtils, checkFilterDcGainEigen) { 
    
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> filter;
    
    double ts = 0.01;
    double cutoff = 10;
    double omega = 2*3.1415*cutoff; // 10 Hz cutoff
    double eps = 1.0;
    
    filter.setOmega(omega);
    filter.setTimeStep(ts);
    filter.setDamping(eps);
    
    filter.reset(Eigen::Vector3d::Zero());
    
    double time = 0;
    
    Eigen::Vector3d input(1,1,1);
    Eigen::Vector3d output;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_TRUE( (output - filter.getOutput()).norm() == 0);
        time += ts;
    }
    
    EXPECT_NEAR((filter.getOutput() - Eigen::Vector3d(1,1,1)).norm(), 0.0, 1e-3);
    
    
}



// TEST_F(testUtils, checkMtLogger){
//     
//     std::vector<std::thread> th;
//     
//     auto f = std::bind(&testUtils::test_mt_log_func, this, std::placeholders::_1);
//     
//     for(int i = 0; i < 5; i++){
//         th.push_back(std::thread(f, i+1));
//     }
//     
//     for(auto& t : th){
//         t.join();
//     }
// }



TEST_F( testUtils, checkLimitedDeque ){
 
    int N = 10;
    
    XBot::Utils::LimitedDeque<int> deque(N);
    
//     EXPECT_THROW( deque.back(), std::out_of_range );
    EXPECT_FALSE( deque.pop_back() );
    EXPECT_TRUE( deque.is_empty() );
    
    for(int i = 0; i < (N-1); i++){
        EXPECT_EQ(deque.size(), i);
        deque.push_back();
        EXPECT_EQ(deque.size(), i+1);
        deque.back() = i;
        EXPECT_FALSE(deque.is_full());
        EXPECT_FALSE(deque.is_empty());
    }
    
    deque.push_back();
    deque.back() = N-1;
    
    EXPECT_TRUE( deque.is_full() );
    EXPECT_EQ( deque.size(), deque.capacity() );
    EXPECT_FALSE( deque.is_empty() );
    
    for(int i = N-1; i >= 0; i--){
        std::cout << deque.back() << std::endl;
        EXPECT_EQ( deque.back(), i );
        EXPECT_TRUE( deque.pop_back() );
        EXPECT_EQ( deque.size(), i );
    }
    
    EXPECT_TRUE( deque.is_empty() );
    
    
    
    for(int i = 0; i < (1.5*N); i++){
        EXPECT_EQ(deque.size(), std::min(N, i));
        deque.push_back();
        EXPECT_EQ(deque.size(), std::min(N, i+1));
        deque.back() = i;
        std::cout << deque.back() << std::endl;
        EXPECT_FALSE(deque.is_empty());
    }
    
    EXPECT_EQ(deque.size(), N);
    
    for(int i = 0; i < N; i++){
        std::cout << "Iter " << i << std::endl;
        EXPECT_EQ(deque.back(), int(1.5*N)-1-i);
        std::cout << deque.back() << std::endl;
        EXPECT_EQ(deque.size(), N-i);
        EXPECT_TRUE(deque.pop_back());
        EXPECT_EQ(deque.size(), N-i-1);
    }
    
    EXPECT_TRUE(deque.is_empty());
    
    
    
    
    
}


TEST_F(testUtils, checkPlanner)
{
    auto logger = XBot::MatLogger::getLogger("/tmp/checkPlannerXBotUtils");
    double x0 = 1;
    double dx0 = -1;
    double ddx0 = 1;
    double goal = 0;
    double x, dx, ddx;
    
    for(double time = 0; time <= 10.0; time += 0.01)
    {
        
        XBot::Utils::FifthOrderPlanning(x0, dx0, ddx0, goal, 0, 10, time, x, dx, ddx);
        
        if(time == 0.0)
        {
            EXPECT_TRUE(std::fabs(x0-x) <= 1e-6);
            EXPECT_TRUE(std::fabs(dx0-dx) <= 1e-6);
            EXPECT_TRUE(std::fabs(ddx0-ddx) <= 1e-6);
        }
        
        logger->add("x", x);
        logger->add("dx", dx);
        logger->add("ddx", ddx);
        logger->add("goal", goal);
    }

    EXPECT_TRUE(std::fabs(goal-x) <= 1e-6);
    EXPECT_TRUE(std::fabs(dx) <= 1e-6);
    EXPECT_TRUE(std::fabs(ddx) <= 1e-6);
    
    logger->flush();
    
    
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}
