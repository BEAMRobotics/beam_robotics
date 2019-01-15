# example_package

This subdirectory contains an example ROS package that demonstrates gtest unit-testing, error reporting, and the separation of functionality between interface-focused ROS-nodelets and processing-focused C++ libraries.

For error/diagnostic reporting the general rule is:
- Errors discovered in ROS (within nodes or nodelets) must be reported using the error logging functions included in ROS, such as ROS_ERROR().
- Errors discovered within internal C++ libraries should report to the ``BOOST_LOG_TRIVIAL(error)`` stream.
The ``BOOST_LOG_TRIVIAL(error)`` stream can then output to a log file so as not to compete with ROS_ERROR for console visibility and to avoid spamming the console.
If a log file is not specified, such as when trying to debug the C++ library outside of ROS, then ``BOOST_LOG_TRIVIAL(error)`` outputs to the console.

Writing to the error stream is easy. Just like std::cout: 

    BOOST_LOG_TRIVIAL(error) << "I'm an error!";
compared to  

    std::cout << "I'm an error!";

Separating the processing-focused C++ libraries from ROS allows easier unit testing of the core processing modules. This package contains basic unit testing for its included example library, which consists of a single class.

## Error
The node has an error in it.
Run the ros node:  

    rosrun example_package example_package_node

Observe that there is an error detected by the ROS side of the system (in the nodelet): 

    [ INFO] [1473384277.014068572]: Initializing nodelet with 8 worker threads. 
    [ERROR] [1473384280.064809589]: Function ExampleClass::Bar() failed!...

Since the ``BOOST_LOG_TRIVIAL(error)`` stream has a log file declared in the nodelet, the error messages reported within the C++ library didn't clog the console. Instead, they were placed into example_package_nodelet.log. To review the log:

    gedit example_package_nodelet.log

If the C++ library is called without the wrapping nodelet, then ``BOOST_LOG_TRIVIAL(error)`` outputs directly to the console for easier debugging. For example, run the example package's tests (build them first using the bash script /autonomoose/scripts/build_and_run_cpp_unit_tests.bash):

    rosrun example_package example_package-test

Observe that while the test set passes, the tests that check for failure produce diagnostic information shown in the console along with the test results.

## Unit Tests
Each package can have many unit tests that verify the internal C++ library's functionality. This package has 2 tests for the library's sole class, ExampleClass.  

The tests currently pass because the FooTest isn't a very good test. If you switch the value of ``NUM_TESTS`` within FooTest according to the comments, then the test may become sufficient to discover the underlying bug within the Foo function.

As is shown in this example package, debugging with diagnostic information being sent to the console can be done comfortably without ROS, BUT without clogging the console if the exact same library is used within ROS and encounters a bug at runtime.

Adding unit tests to the catkin build process is done by adding the relevant lines to your package's ``CMakeLists.txt`` file. Check out the lines at the end of this package's ``CMakeLists.txt`` file and the test source files contained in the package's ``/test`` folder to see how you do this. Once tests are declared here, travis will call the script /autonomoose/scripts/build_and_run_cpp_unit_tests.bash to run them and they will be integrated into our build/continuous-integration process.
