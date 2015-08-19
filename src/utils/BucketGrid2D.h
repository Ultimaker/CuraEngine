/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_BUCKET_GRID_2D_H
#define UTILS_BUCKET_GRID_2D_H

#include "logoutput.h"
#include "intpoint.h"
#include <unordered_map>

namespace cura
{

/*!
 * Container for items with location for which the lookup for nearby items is optimized.
 * 
 * It functions by hashing the items location and lookuping up based on the hash of that location and the hashes of nearby locations.
 */
template<typename T>
class BucketGrid2D
{
private:

    /*!
     * Returns a point for which the hash is at a grid position of \p relativeHash relative to \p p.
     * 
     * \param p The point for which to get the relative point to hash
     * \param relativeHash The relative position - in grid terms - of the relative point. 
     * \return A point for which the hash is at a grid position of \p relativeHash relative to \p p.
     */
    inline Point getRelativeForHash(const Point& p, const Point& relativeHash)
    {
        return p + relativeHash*squareSize;
    }


    /*!
     * A hash class representing the hash function object.
     */
    struct PointHasher
    {

        /*!
         * The basic hash function for a 2D grid position.
         * \param p The grid location to hash
         * \return the hash
         */
        inline uint32_t pointHash_simple(const Point& p) const
        {
            return p.X ^ (p.Y << 8);
        }

        /*!
         * The hash function for a 2D position.
         * \param point The location to hash
         * \return the hash
         */
        inline uint32_t pointHash(const Point& point) const
        {
            Point p = point/squareSize;
            return pointHash_simple(p);
        }
/*
        inline uint32_t pointHash(const Point& point, const Point& relativeHash) const
        {
            Point p = p/squareSize + relativeHash;
            return pointHash_simple(p);
        }*/

        /*!
         * The horizontal and vertical size of a cell in the grid; the width and height of a bucket.
         */
        int squareSize;
        
        /*!
         * Basic constructor.
         * \param squareSize The horizontal and vertical size of a cell in the grid; the width and height of a bucket.
         */
        PointHasher(int squareSize) : squareSize(squareSize) {};
        
        /*!
         * See PointHasher::pointHash
         */
        uint32_t operator()(const Point& p) const { return pointHash(p); };

    };

    /*!
     * A helper predicate object which allways returns false when comparing two objects.
     * 
     * This is used for mapping each point to a unique object, even when two objects have the same point associated with it.
     */
    struct NeverEqual
    {
        template<typename S>
        bool operator()(const S& p1, const S& p2) const { return false; };
    };


private:
    /*!
     * Basic constructor.
     * \param squareSize The horizontal and vertical size of a cell in the grid; the width and height of a bucket.
     */
    int squareSize;
    /*!
     * The map type used to associate points with their objects.
     */
    typedef typename std::unordered_map<Point, T, PointHasher, NeverEqual> Map;
    
    /*!
     * The map used to associate points with their objects.
     */
    Map point2object;


public:
    /*!
     * The constructor for a bucket grid.
     * 
     * \param squareSize The horizontal and vertical size of a cell in the grid; the width and height of a bucket.
     * \param initial_map_size The minimal number of initial buckets 
     */
    BucketGrid2D(int squareSize, unsigned int initial_map_size = 4) : squareSize(squareSize), point2object(initial_map_size, PointHasher(squareSize)) {};

    /*!
     * Find all objects with a point in a grid cell at a distance of one cell from the cell of \p p.
     * 
     * \warning Objects may occur multiple times in the output!
     * 
     * \param p The point for which to find close points.
     * \param ret Ouput parameter: all objects close to \p p.
     */
    void findNearbyObjects(Point& p, std::vector<T>& ret)
    {
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int bucket_idx = point2object.bucket(getRelativeForHash(p, Point(x,y))); // when the hash is not a hash of a present item, the bucket_idx returned may be one already encountered
                for ( auto local_it = point2object.begin(bucket_idx); local_it!= point2object.end(bucket_idx); ++local_it )
                {
                    ret.push_back(local_it->second);
                }
            }
        }
    };
    
    /*!
     * Find all objects with a point in a grid cell at a distance of one cell from the cell of \p p.
     * 
     * \warning Objects may occur multiple times in the output!
     * 
     * \param p The point for which to find close points.
     * \return All objects close to \p p.
     */
    std::vector<T> findNearbyObjects(Point& p)
    {
        std::vector<T> ret;
        findNearbyObjects(p, ret);
        return ret;
    }

    /*!
     * Find the nearest object to a given lcoation \p p, if there is any in a neighboring cell in the grid.
     * 
     * \param p The point for which to find the nearest object.
     * \param nearby Output parameter: the nearest object, if any
     * \return Whether an object has been found.
     */
    bool findNearestObject(Point& p, T& nearby)
    {
        bool found = false;
        int64_t bestDist2 = squareSize*9; // 9 > sqrt(2*2 + 2*2)^2  which is the square of the largest distance of a point to a point in a neighboring cell
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int bucket_idx = point2object.bucket(getRelativeForHash(p, Point(x,y)));
                for ( auto local_it = point2object.begin(bucket_idx); local_it!= point2object.end(bucket_idx); ++local_it )
                {
                    int32_t dist2 = vSize2(local_it->first - p);
                    if (dist2 < bestDist2)
                    {
                        found = true;
                        nearby = local_it->second;
                        bestDist2 = dist2;
                    }
                }
            }
        }
        return found;
    };

    
    /*!
     * Insert a new point into the bucket grid.
     * 
     * \param p The location associated with \p t.
     * \param t The object to insert in the grid cell for position \p p.
     */
    void insert(Point& p, T& t)
    {
//         typedef typename Map::iterator iter;
//         std::pair<iter, bool> emplaced = 
        point2object.emplace(p, t);
//         if (! emplaced.second)
//             logError("Error! BucketGrid2D couldn't insert object!");
    };





};

}//namespace cura
#endif//BUCKET_GRID_2D_H
