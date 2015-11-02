//Copyright (c) 2015 Ultimaker B.V.
//UltiScanTastic is released under the terms of the AGPLv3 or higher.

#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>

/*!
 * \brief Runs the test cases.
 */
int main(int argc,char** argv)
{
    CppUnit::TextUi::TestRunner runner;
    CppUnit::TestFactoryRegistry& registry = CppUnit::TestFactoryRegistry::getRegistry();
    runner.addTest(registry.makeTest());
    //return runner.run("",false);
    bool success = runner.run("", false);
    return success ? 0 : 1;
}

