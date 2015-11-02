//Copyright (c) 2015 Ultimaker B.V.
//UltiScanTastic is released under the terms of the AGPLv3 or higher.

#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/XmlOutputter.h>

/*!
 * \brief Runs the test cases.
 */
int main(int argc,char** argv)
{
    CppUnit::TextUi::TestRunner runner;
    
    //Set the output type to be JUnit-style XML.
    std::ofstream output("output.xml");
    CppUnit::XmlOutputter* outputter = new CppUnit::XmlOutputter(&runner.result(), output);
    runner.setOutputter(outputter);
    
    CppUnit::TestFactoryRegistry& registry = CppUnit::TestFactoryRegistry::getRegistry();
    runner.addTest(registry.makeTest()); //Makes a test suite from all test cases that are registered with CPPUNIT_TEST_SUITE_REGISTRATION().
    bool success = runner.run("", false); //Run the tests!
    return success ? 0 : 1;
}

