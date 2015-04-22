Code Conventions
=======
Note that the code convention described here have not all yet been fully implemented.

Bracketing and indenting
-----
~~~~~~~~~~~~~~~{.cpp}
if (condition) // brackets always on new lines
{ // allways a bracket after an if, for, while, etc.
    // indent always with 4 spaces, never with tabs
}
else // else on new line
{
    // more code
}
~~~~~~~~~~~~~~~

Naming conventions
------
 * variables: lower_case_with_underscores
 * functions: loweCamelCase
 * classes: UpperCamelCase
 * macros: UPPER_CASE_WITH_UNDERSCORES
~~~~~~~~~~~~~~~{.cpp}
#define UPPER_CASE_MACRO 1

class UpperCamelCase
{
    MemberVariableObject with_underscores;
    
    // start with input variable(s) and end with output variable(s)
    void lowerCamelCaseFunctions(ParamObject& also_with_underscores)
    {
        LocalObject under_scores;
    }
};
~~~~~~~~~~~~~~~

Ordering
----
~~~~~~~~~~~~~~~{.cpp}
class Example
{   
    // start with input variable(s) and end with output parameter(s) 
    void function1(ParamObject& input_variable, int setting_parameter, ParamObject2& return_parameter)
    {
        function2();
        function3();
    }
    
    // place functions called solely by one other function below it chronologically
    void function2();
    
    void function3();
};
~~~~~~~~~~~~~~~

Documentation
----
We use [Doxygen](www.doxygen.org/) to generate documentation. Try to keep your documentation in doxygen style.

Here's a small example:
~~~~~~~~~~~~~~~{.cpp}
/ *!
 * Doxygen style comments!
 *
 * \param param1 explanation may refer to another \p param2
 * /
void function(int param1, int param2)
{
    // non-doxygen style comments on implementation details
}

int member; //!< inline doxygen comment on the entry to the left
~~~~~~~~~~~~~~~

Files
--------
For a file Foo.h (UpperCamelCase):
~~~~~~~~~~~~~~~{.cpp}
#ifndef FOO_H
#define FOO_H
// [content]
#endif//FOO_H
~~~~~~~~~~~~~~~


Other
----
~~~~~~~~~~~~~~~{.cpp}
#include all
#include includes
#include on
#include top

enum class EnumExample 
{
    ELEM0 = 0,
    ELEM1 = 1
}
~~~~~~~~~~~~~~~

Illegal syntax
----
~~~~~~~~~~~~~~~{.cpp}
void function()
{
    if (condition)
        single_line_outside_code_block(); // always use braces!
}; // unneccesary semicolon after function definition is not allowed
~~~~~~~~~~~~~~~
