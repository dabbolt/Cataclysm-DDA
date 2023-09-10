#pragma once
#ifndef CATA_SRC_FLOOD_FILL_H
#define CATA_SRC_FLOOD_FILL_H

#include <queue>
#include <vector>
#include <unordered_set>

#include "enums.h"
#include "point.h"

namespace ff
{
/**
* Given a starting point, flood fill out to the 4-connected points, applying the provided predicate
* to determine if a given point should be added to the collection of flood-filled points, and then
* return that collection.
* @param starting_point starting point of the flood fill. No assumptions made about if it will satisfy
* the predicate.
* @param visited externally provided set of points that have already been designated as visited which
* will be updated by this call.
* @param predicate UnaryPredicate that will be provided with a point for evaluation as to whether or
* not the point should be filled.
*/
template<typename Point, typename UnaryPredicate>
std::vector<Point> point_flood_fill_4_connected( const Point &starting_point,
        std::unordered_set<Point> &visited, UnaryPredicate predicate )
{
    std::vector<Point> filled_points;
    std::queue<Point> to_check;
    to_check.push( starting_point );
    while( !to_check.empty() ) {
        const Point current_point = to_check.front();
        to_check.pop();

        if( visited.find( current_point ) != visited.end() ) {
            continue;
        }

        visited.emplace( current_point );

        if( predicate( current_point ) ) {
            filled_points.emplace_back( current_point );
            to_check.push( current_point + point_south );
            to_check.push( current_point + point_north );
            to_check.push( current_point + point_east );
            to_check.push( current_point + point_west );
        }
    }

    return filled_points;
}

/**
* Given a starting point, flood fill out to the 26-connected points, applying the provided predicate
* to determine if a given point should be added to the collection of flood-filled points, and then
* return that collection.
* @param starting_point starting point of the flood fill. No assumptions made about if it will satisfy
* the predicate.
* @param predicate UnaryPredicate that will be provided with a point for evaluation as to whether or
* not the point should be filled.
* @param visitor UnaryVisitor that will be provided with a point to act on.
*
* TODO: This is wildly suboptimal in several ways. After profiling, *when* a hotspot shows up
* in here, the next step is to convert it into a scanline flood fill.
* to make it z-level aware, you might push scanlines for other z-levels into
* one queue for each level, then when you're done with level X, you proceed to either X+1 or X-1,
* keeping in mind you might recurse back down to an "island" on the previous z-level later >_<
*/
template<typename UnaryPredicate, typename UnaryVisitor>
void flood_fill_visit_26_connected( const tripoint &starting_point, UnaryPredicate predicate,
                                    UnaryVisitor visitor )
{
    std::queue<tripoint> to_check;
    std::queue<tripoint> to_check_up;
    std::queue<tripoint> to_check_down;
    std::unordered_set<tripoint> visited;
    to_check.push( starting_point );
    while( true ) {
        int vertical_direction = 0;
        tripoint current_point;
        if( !to_check.empty() ) {
            current_point = to_check.front();
            to_check.pop();
        } else if( !to_check_up.empty() ) {
            current_point = to_check_up.front();
            vertical_direction = 1;
            to_check_up.pop();
        } else if( !to_check_down.empty() ) {
            current_point = to_check_down.front();
            vertical_direction = -1;
            to_check_down.pop();
        } else {
            break;
        }
        if( visited.find( current_point ) != visited.end() ) {
            continue;
        }

        visited.emplace( current_point );

        if( predicate( current_point, vertical_direction ) ) {
            visitor( current_point );
            for( const tripoint &h_offset : eight_horizontal_neighbors ) {
                to_check.push( current_point + h_offset );
            }
            to_check_up.push( current_point + tripoint_above );
            to_check_down.push( current_point + tripoint_below );
        }
    }
}

struct span {
    span() : startX( 0 ), endX( 0 ), y( 0 ), dy( 0 ), z( 0 ),
        dz( 0 ) {}
    span( uint8_t p_startX, uint8_t p_endX, uint8_t p_y, int8_t p_dy, int8_t p_z,
          int8_t p_dz ) : startX( p_startX ), endX( p_endX ), y( p_y ), dy( p_dy ), z( p_z ),
        dz( p_dz ) {}

    uint8_t startX;
    uint8_t endX;
    uint8_t y;
    int8_t dy;
    int8_t z;
    int8_t dz;
};

template<typename UnaryPredicate, typename UnaryVisitor>
void flood_fill_visit_10_connected( const tripoint &starting_point, UnaryPredicate predicate,
                                    UnaryVisitor visitor )
{
    std::unordered_set<tripoint> visited;
    std::unordered_set<tripoint> visited_vertically;
    //std::array<std::bitset, OVERMAP_LAYERS> visited;
    std::array<std::vector<span>, OVERMAP_LAYERS> spans_to_process;
    int current_z = starting_point.z;
    spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( starting_point.x, starting_point.x,
            starting_point.y, 1,
            starting_point.z, 0 );
    spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( starting_point.x, starting_point.x,
            starting_point.y - 1,
            -1, starting_point.z, 0 );
    static int check_count = 0;
    check_count = 0;
    static int visit_count = 0;
    visit_count = 0;
    auto check = [&predicate, &visited]( const tripoint loc, int dir ) {
        check_count++;
        return visited.find( loc ) == visited.end() && predicate( loc, dir );
    };
    auto check_vertical = [&predicate, &visited_vertically]( const tripoint loc, int dir ) {
        check_count++;
        return visited_vertically.find( loc ) == visited_vertically.end() && predicate( loc, dir );
    };
    while( true ) {
        struct span current_span;
        if( !spans_to_process[current_z + OVERMAP_DEPTH].empty() ) {
            current_span = spans_to_process[current_z + OVERMAP_DEPTH].back();
            spans_to_process[current_z + OVERMAP_DEPTH].pop_back();
        } else {
            for( current_z = OVERMAP_HEIGHT; current_z >= -OVERMAP_DEPTH; current_z-- ) {
                if( !spans_to_process[current_z + OVERMAP_DEPTH].empty() ) {
                    current_span = spans_to_process[current_z + OVERMAP_DEPTH].back();
                    spans_to_process[current_z + OVERMAP_DEPTH].pop_back();
                    break;
                }
            }
            if( current_z < -OVERMAP_DEPTH ) {
                break;
            }
        }
        tripoint current_point{ current_span.startX, current_span.y, current_span.z };
        // Special handling for spans with a vertical offset.
        if( current_span.dz != 0 ) {
            bool span_found = 0;
            struct span new_span {
                current_span.startX, current_span.endX, current_span.y, current_span.dy, current_span.z, 0
            };
            // Iterate over tiles in the span, adding to new span as long as check passes.
            while( current_point.x <= current_span.endX ) {
                if( check_vertical( current_point, current_span.dz ) ) {
                    span_found = true;
                    new_span.endX = current_point.x;
                    visited_vertically.insert( current_point );
                } else {
                    if( span_found ) {
                        spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( new_span.startX, new_span.endX,
                                new_span.y, 1, current_point.z, 0 );
                        spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( new_span.startX, new_span.endX,
                                new_span.y - 1, -1, current_point.z, 0 );
                    }
                    span_found = false;
                }
                current_point.x++;
            }
            continue;
        }

        // Scan to the left of the leftmost point in the current span.
        if( check( current_point, 0 ) ) {
            // This decrements before visiting because the next loop will visit the current value of startX
            current_point.x--;
            while( check( current_point, 0 ) ) {
                visit_count++;
                visitor( current_point );
                visited.insert( current_point );
                current_point.x--;
            }
            current_point.x++;
            // If we found visitable tiles to the left of the span, emit a new span going in the other y direction to go around corners.
            if( current_point.x < current_span.startX ) {
                spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( current_point.x - 1,
                        current_span.startX - 1, current_span.y - current_span.dy, -current_span.dy, current_point.z, 0 );
            }
        }
        int furthestX = current_point.x;
        current_point.x = current_span.startX;
        // Scan the span itself, running off the edge to the right if possible.
        while( current_point.x <= current_span.endX ) {
            while( check( current_point, 0 ) ) {
                visit_count++;
                visitor( current_point );
                visited.insert( current_point );
                current_point.x++;
            }
            // If we have made any progress in the above loops, emit a span going in our initial y direction as well as in both vertical directions.
            if( current_point.x > furthestX ) {
                spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( furthestX - 1, current_point.x,
                        current_span.y + current_span.dy, current_span.dy,
                        current_span.z, 0 );
                if( current_z + 1 < OVERMAP_LAYERS ) {
                    spans_to_process[current_z + OVERMAP_DEPTH + 1].emplace_back( furthestX, current_point.x - 1,
                            current_span.y, 0, current_span.z + 1, 1 );
                }
                if( current_z - 1 >= 0 ) {
                    spans_to_process[current_z + OVERMAP_DEPTH - 1].emplace_back( furthestX, current_point.x - 1,
                            current_span.y, 0, current_span.z - 1, -1 );
                }
            }
            // If we found visitable tiles to the right of the span, emit a new span going in the other y direction to go around corners.
            if( current_point.x - 1 > current_span.endX ) {
                spans_to_process[current_z + OVERMAP_DEPTH].emplace_back( current_span.endX + 1, current_point.x,
                        current_span.y - current_span.dy, -current_span.dy, current_span.z, 0 );
            }
            // This is pointing to a tile that faied the predicate, so advance to the next tile.
            current_point.x++;
            // Scan past any unvisitable tiles up to the end of the current span.
            while( current_point.x < current_span.endX && !predicate( current_point, 0 ) ) {
                current_point.x++;
            }
            // We either found a new visitable tile or ran off the end of the span, record our new scan start point regardless.
            furthestX = current_point.x;
        }
    }
}
} // namespace ff

#endif // CATA_SRC_FLOOD_FILL_H
