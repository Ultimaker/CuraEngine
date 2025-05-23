#include <gtest/gtest.h>

// Includes for CuraEngine classes
#include "pathPlanning/Comb.h"
#include "sliceDataStorage.h"
#include "ExtruderTrain.h"
#include "settings/Settings.h"
#include "settings/EnumSettings.h"
#include "utils/polygon.h" // For creating test polygons
#include "geometry/Point.h" // For cura::Point / cura::Point2LL
#include "geometry/Polygon.h" // For cura::Polygon / cura::Polygons / cura::ConstPolygonRef
#include "geometry/Shape.h" // For cura::Shape
#include "sliceDataStorage.h" // For cura::SliceLayer, cura::SliceLayerPart, cura::Walls
#include "Application.h"      // For cura::Application and cura::Scene


// Helper function to create simple SliceDataStorage with walls
cura::SliceDataStorage createTestData(
    cura::coord_t object_size,
    cura::coord_t wall_thickness,
    int num_walls,
    cura::LayerIndex layer_nr = 0) 
{
    cura::SliceDataStorage storage;
    storage.model_min = cura::Point3(-object_size / 2, -object_size / 2, 0);
    storage.model_max = cura::Point3(object_size / 2, object_size / 2, object_size);
    storage.model_size = storage.model_max - storage.model_min;
    storage.scene_min = storage.model_min;
    storage.scene_max = storage.model_max;
    storage.scene_size = storage.model_size;

    cura::SliceLayer layer(layer_nr, 0, object_size); 
    
    cura::SliceLayerPart part; 

    cura::Polygon outer_wall_poly_base;
    outer_wall_poly_base.emplace_back(object_size / 2, object_size / 2);
    outer_wall_poly_base.emplace_back(object_size / 2, -object_size / 2);
    outer_wall_poly_base.emplace_back(-object_size / 2, -object_size / 2);
    outer_wall_poly_base.emplace_back(-object_size / 2, object_size / 2);
    
    part.outline.add(outer_wall_poly_base);

    cura::Walls walls_data; 
    walls_data.outer_wall_polys.add(outer_wall_poly_base);

    if (num_walls > 1) {
        cura::Polygon second_wall_poly_base;
        cura::coord_t offset = wall_thickness; 
        second_wall_poly_base.emplace_back(object_size / 2 - offset, object_size / 2 - offset);
        second_wall_poly_base.emplace_back(object_size / 2 - offset, -object_size / 2 + offset);
        second_wall_poly_base.emplace_back(-object_size / 2 + offset, -object_size / 2 + offset);
        second_wall_poly_base.emplace_back(-object_size / 2 + offset, object_size / 2 - offset);
        walls_data.second_wall_polys.add(second_wall_poly_base);
    }
    
    part.wall_parts.push_back(walls_data); 
    layer.parts.push_back(part);
    storage.layers.emplace_back(std::move(layer)); 

    return storage;
}


// Test fixture for Combing tests
class CombPolygonTest : public ::testing::Test {
protected:
    cura::Settings global_settings_; 
    cura::ExtruderTrain train{&global_settings_, &global_settings_, cura::ExtruderTrain::ExtruderNr(0)};
    
    cura::CombPaths comb_paths;
    bool unretract_before_last_travel_move = false;
    cura::coord_t object_size = 100 * cura::MM2INT; 
    cura::coord_t wall_thickness = 1 * cura::MM2INT; 

    void SetUp() override {
        train.settings_.set<cura::CombingMode>("combing_mode", cura::CombingMode::ALL);
        train.settings_.set<bool>("travel_avoid_other_parts", false);
        train.settings_.set<cura::coord_t>("travel_avoid_distance", 5 * cura::MM2INT); 
        train.settings_.set<cura::coord_t>("retraction_combing_max_distance", 1000 * cura::MM2INT);
        train.settings_.set<cura::coord_t>("travel_retract_before_outer_wall_distance", 50); // Small default
        train.settings_.set<cura::coord_t>("offset_from_outlines", wall_thickness / 2); 
        train.settings_.set<cura::coord_t>("move_inside_distance", wall_thickness / 4); 
    }

    void TearDown() override {
    }

    bool isPointApproximatelyInsideShape(const cura::Point2LL& pt, const cura::Shape& shape, cura::coord_t tolerance = 50) {
        if (shape.empty()) return false;
        for (const cura::ConstPolygonRef poly : shape) {
            if (poly.inside(pt, true)) { 
                return true;
            }
            for (size_t i = 0; i < poly.size(); ++i) {
                cura::Point2LL p1 = poly[i];
                cura::Point2LL p2 = poly[(i + 1) % poly.size()];
                if (LinearAlg2D::getDistSqToLineSegment(pt, p1, p2) < tolerance * tolerance) {
                    return true;
                }
            }
        }
        return false;
    }
};

TEST_F(CombPolygonTest, CombOuterWall) {
    train.settings_.set<cura::CombingPolygonType>("combing_polygon_type", cura::CombingPolygonType::OUTER_WALL);
    cura::SliceDataStorage storage = createTestData(object_size, wall_thickness, 1, 0);
    
    ASSERT_FALSE(storage.layers.empty());
    ASSERT_FALSE(storage.layers[0].parts.empty());
    ASSERT_FALSE(storage.layers[0].parts[0].wall_parts.empty());

    cura::Shape comb_boundary_min = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;
    cura::Shape comb_boundary_opt = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;

    cura::Comb comber(storage, 0, comb_boundary_min, comb_boundary_opt, 
                      train.settings_.get<cura::coord_t>("offset_from_outlines"), 
                      train.settings_.get<cura::coord_t>("travel_avoid_distance"),
                      train.settings_.get<cura::coord_t>("move_inside_distance"));

    cura::Point2LL start_point(0, 0); 
    cura::Point2LL end_point(object_size / 2 - wall_thickness, 0); 

    bool success = comber.calc(false, false, train, start_point, end_point, comb_paths, true, true, 0, unretract_before_last_travel_move);
    ASSERT_TRUE(success) << "Combing calculation failed for OuterWall.";
    ASSERT_FALSE(comb_paths.empty()) << "No combing paths generated for OuterWall.";

    for (const auto& path_segment : comb_paths) {
        for (const cura::Point2LL& pt : path_segment) {
            EXPECT_TRUE(isPointApproximatelyInsideShape(pt, comb_boundary_opt, wall_thickness)) 
                << "Point " << pt << " is outside the expected OuterWall boundary.";
        }
    }
}

TEST_F(CombPolygonTest, CombSecondWall_Exists) {
    train.settings_.set<cura::CombingPolygonType>("combing_polygon_type", cura::CombingPolygonType::SECOND_WALL);
    cura::SliceDataStorage storage = createTestData(object_size, wall_thickness, 2, 0); // 2 walls
        
    ASSERT_FALSE(storage.layers.empty());
    ASSERT_FALSE(storage.layers[0].parts.empty());
    ASSERT_FALSE(storage.layers[0].parts[0].wall_parts.empty());
    ASSERT_FALSE(storage.layers[0].parts[0].wall_parts[0].second_wall_polys.empty());

    cura::Shape initial_comb_boundary_min = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;
    cura::Shape initial_comb_boundary_opt = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;

    cura::Comb comber(storage, 0, initial_comb_boundary_min, initial_comb_boundary_opt,
                      train.settings_.get<cura::coord_t>("offset_from_outlines"), 
                      train.settings_.get<cura::coord_t>("travel_avoid_distance"),
                      train.settings_.get<cura::coord_t>("move_inside_distance"));
    
    cura::Point2LL start_point(0, 0); 
    cura::Point2LL end_point(object_size / 2 - wall_thickness * 1.5, 0); 

    bool success = comber.calc(false, false, train, start_point, end_point, comb_paths, true, true, 0, unretract_before_last_travel_move);
    ASSERT_TRUE(success) << "Combing calculation failed for SecondWall.";
    ASSERT_FALSE(comb_paths.empty()) << "No combing paths generated for SecondWall.";

    const cura::Shape& second_wall_shape = storage.layers[0].parts[0].wall_parts[0].second_wall_polys;
    for (const auto& path_segment : comb_paths) {
        for (const cura::Point2LL& pt : path_segment) {
            EXPECT_TRUE(isPointApproximatelyInsideShape(pt, second_wall_shape, wall_thickness)) 
                << "Point " << pt << " is outside the expected SecondWall boundary.";
        }
    }
}

TEST_F(CombPolygonTest, CombSecondWall_FallbackWhenNoSecondWall) {
    train.settings_.set<cura::CombingPolygonType>("combing_polygon_type", cura::CombingPolygonType::SECOND_WALL);
    cura::SliceDataStorage storage = createTestData(object_size, wall_thickness, 1, 0); 
    
    ASSERT_FALSE(storage.layers.empty());
    ASSERT_FALSE(storage.layers[0].parts.empty());
    ASSERT_FALSE(storage.layers[0].parts[0].wall_parts.empty());
    ASSERT_TRUE(storage.layers[0].parts[0].wall_parts[0].second_wall_polys.empty()); 

    cura::Shape initial_comb_boundary_min = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;
    cura::Shape initial_comb_boundary_opt = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;

    cura::Comb comber(storage, 0, initial_comb_boundary_min, initial_comb_boundary_opt,
                      train.settings_.get<cura::coord_t>("offset_from_outlines"), 
                      train.settings_.get<cura::coord_t>("travel_avoid_distance"),
                      train.settings_.get<cura::coord_t>("move_inside_distance"));

    cura::Point2LL start_point(0, 0);
    cura::Point2LL end_point(object_size / 2 - wall_thickness, 0); 

    bool success = comber.calc(false, false, train, start_point, end_point, comb_paths, true, true, 0, unretract_before_last_travel_move);
    ASSERT_TRUE(success) << "Combing calculation failed for SecondWall fallback.";
    ASSERT_FALSE(comb_paths.empty()) << "No combing paths generated for SecondWall fallback.";

    const cura::Shape& outer_wall_shape = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;
    for (const auto& path_segment : comb_paths) {
        for (const cura::Point2LL& pt : path_segment) {
            EXPECT_TRUE(isPointApproximatelyInsideShape(pt, outer_wall_shape, wall_thickness)) 
                << "Point " << pt << " is outside the OuterWall boundary after SecondWall fallback.";
        }
    }
}

TEST_F(CombPolygonTest, CombOutline) {
    train.settings_.set<cura::CombingPolygonType>("combing_polygon_type", cura::CombingPolygonType::OUTLINE);
    cura::SliceDataStorage storage = createTestData(object_size, wall_thickness, 1, 0); 
    
    ASSERT_FALSE(storage.layers.empty());
    ASSERT_FALSE(storage.layers[0].parts.empty());
    ASSERT_FALSE(storage.layers[0].parts[0].outline.empty());

    cura::Shape initial_comb_boundary_min = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;
    cura::Shape initial_comb_boundary_opt = storage.layers[0].parts[0].wall_parts[0].outer_wall_polys;

    cura::Comb comber(storage, 0, initial_comb_boundary_min, initial_comb_boundary_opt,
                      train.settings_.get<cura::coord_t>("offset_from_outlines"), 
                      train.settings_.get<cura::coord_t>("travel_avoid_distance"),
                      train.settings_.get<cura::coord_t>("move_inside_distance"));

    cura::Point2LL start_point(0, 0);
    cura::Point2LL end_point(object_size / 2 - wall_thickness / 4, 0); 

    bool success = comber.calc(false, false, train, start_point, end_point, comb_paths, true, true, 0, unretract_before_last_travel_move);
    ASSERT_TRUE(success) << "Combing calculation failed for Outline.";
    ASSERT_FALSE(comb_paths.empty()) << "No combing paths generated for Outline.";
    
    const cura::Shape& layer_outline_shape = storage.layers[0].parts[0].outline; 
    for (const auto& path_segment : comb_paths) {
        for (const cura::Point2LL& pt : path_segment) {
            EXPECT_TRUE(isPointApproximatelyInsideShape(pt, layer_outline_shape, wall_thickness))
                << "Point " << pt << " is outside the expected LayerOutline boundary.";
        }
    }
}
