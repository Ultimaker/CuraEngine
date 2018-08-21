//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef VELOCITY_H
#define VELOCITY_H

#include <algorithm> //For std::max and std::min.

namespace cura
{

/*
 * Represents a velocity in millimetres per second.
 *
 * This is a facade. It behaves like a double, only it can't be negative.
 */
struct Velocity
{
    /*
     * \brief Default constructor setting velocity to 0.
     */
    Velocity() : value(0.0) {};

    /*
     * \brief Casts a double to a Velocity instance.
     */
    Velocity(double value) : value(std::max(value, 0.0)) {};

    /*
     * \brief Casts the Temperature instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * Some operators for arithmetic on velocities.
     */
    Velocity operator *(const Velocity& other) const
    {
        return Velocity(value * other.value);
    }
    template<typename E> Velocity operator *(const E& other) const
    {
        return Velocity(value * other);
    }
    Velocity operator /(const Velocity& other) const
    {
        return Velocity(value / other.value);
    }
    template<typename E> Velocity operator /(const E& other) const
    {
        return Velocity(value / other);
    }
    Velocity& operator *=(const Velocity& other)
    {
        value *= other.value;
        return *this;
    }
    template<typename E> Velocity& operator *=(const E& other)
    {
        value *= other;
        return *this;
    }
    Velocity& operator /=(const Velocity& other)
    {
        value /= other.value;
        return *this;
    }
    template<typename E> Velocity& operator /=(const E& other)
    {
        value /= other;
        return *this;
    }

    /*
     * \brief The actual temperature, as a double.
     */
    double value = 0;
};

}

#endif //VELOCITY_H