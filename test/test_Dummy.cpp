#include <boost/test/unit_test.hpp>
#include <camera_spinnaker/Dummy.hpp>

using namespace camera_spinnaker;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    camera_spinnaker::DummyClass dummy;
    dummy.welcome();
}
