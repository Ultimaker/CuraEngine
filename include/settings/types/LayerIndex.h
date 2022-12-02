//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LAYERINDEX_H
#define LAYERINDEX_H

#include <functional>

namespace cura
{

/*
 * \brief Struct behaving like a layer number.
 *
 * This is a facade. It behaves exactly like an integer but is used to indicate
 * that it is a layer number.
 */
struct LayerIndex
{
    /*
     * \brief Default constructor setting the layer index to 0.
     */
    constexpr LayerIndex() : value(0) {};

    /*
     * \brief Casts an integer to a LayerIndex instance.
     */
    constexpr LayerIndex(int value) : value(value) {};

    /*
     * \brief Casts the LayerIndex instance to an integer.
     */
    constexpr operator int() const
    {
        return value;
    }

    /*
     * Some operators to add and subtract layer numbers.
     */
    LayerIndex operator +(const LayerIndex& other) const
    {
        return LayerIndex(value + other.value);
    }
    template<typename E> LayerIndex operator +(const E& other) const
    {
        return LayerIndex(value + other);
    }

    LayerIndex operator -(const LayerIndex& other) const
    {
        return LayerIndex(value - other.value);
    }
    template<typename E> LayerIndex operator -(const E& other) const
    {
        return LayerIndex(value - other);
    }

    LayerIndex& operator +=(const LayerIndex& other)
    {
        value += other.value;
        return *this;
    }
    template<typename E> LayerIndex& operator +=(const E& other)
    {
        value += other;
        return *this;
    }

    LayerIndex& operator -=(const LayerIndex& other)
    {
        value -= other.value;
        return *this;
    }
    template<typename E> LayerIndex& operator -=(const E& other)
    {
        value -= other;
        return *this;
    }

    LayerIndex& operator ++()
    {
        value++;
        return *this;
    }
    LayerIndex operator ++(int) //Postfix.
    {
        LayerIndex original_value(value);
        operator++(); //Increment myself.
        return original_value;
    }
    LayerIndex& operator --()
    {
        value--;
        return *this;
    }
    LayerIndex operator --(int) //Postfix.
    {
        LayerIndex original_value(value);
        operator--(); //Decrement myself.
        return original_value;
    }

    /*
     * \brief The actual layer index.
     *
     * Note that this could be negative for raft layers.
     */
    int value = 0;
};

}

namespace std
{
    template<>
    struct hash<cura::LayerIndex>
    {
        size_t operator()(const cura::LayerIndex& layer_index) const
        {
            return hash<int>()(layer_index.value);
        }
    };
}

#endif //LAYERINDEX_H
