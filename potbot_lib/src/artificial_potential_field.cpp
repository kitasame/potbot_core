#include <potbot_lib/artificial_potential_field.h>

namespace potbot_lib{

    ArtificialPotentialField::ArtificialPotentialField(size_t rows, size_t cols, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field, double field_origin_x, double field_origin_y)
    // : potential::Field::Field(rows, cols, resolution)
    {
        weight_attraction_field_                = weight_attraction_field;
        weight_repulsion_field_                 = weight_repulsion_field;
        distance_threshold_repulsion_field_     = distance_threshold_repulsion_field;
        initPotentialField(rows, cols, resolution, field_origin_x, field_origin_y);
    }

    void ArtificialPotentialField::initPotentialField(size_t rows, size_t cols, double resolution, double field_origin_x, double field_origin_y)
    {
        initField(rows, cols, resolution, field_origin_x, field_origin_y);
    }

    void ArtificialPotentialField::setGoal(size_t index)
    {
        setFieldInfo(index, potential::GridInfo::IS_GOAL, true);

        potential::FieldGrid val = getValue(index);
        setFieldInfo(index, potential::GridInfo::IS_AROUND_GOAL, true);

        std::vector<size_t> search_indexes;
        getSquareIndex(search_indexes, val.row, val.col, 1);
        for (const auto&  i:search_indexes)
        {
            setFieldInfo(i, potential::GridInfo::IS_AROUND_GOAL, true);
        }
    }

    void ArtificialPotentialField::setRobot(size_t index)
    {
        setFieldInfo(index, potential::GridInfo::IS_ROBOT, true);
    }

    void ArtificialPotentialField::setObstacle(size_t index)
    {
        setFieldInfo(index, potential::GridInfo::IS_OBSTACLE, true);
    }

    void ArtificialPotentialField::clearObstacles()
    {
        for (const auto& o:obstacles_)
        {
            try 
            {
                setFieldInfo(getFieldIndex(o), potential::GridInfo::IS_OBSTACLE, false);
            }
            catch(...){}
        }
        obstacles_.clear();
    }

    void ArtificialPotentialField::setParams(double wa, double wr, double dtr)
    {
        weight_attraction_field_ = wa;
        weight_attraction_field_ = wr;
        distance_threshold_repulsion_field_ = dtr;
    }

    void ArtificialPotentialField::setGoal(double x, double y)  //目標ゴールを設定している
    {
        goal_ = Point{x,y};
        try 
        {
            setGoal(getFieldIndex(x,y));
        }
        catch(...){}
    }

    void ArtificialPotentialField::setRobot(double x, double y) //ロボットの位置情報を設定している
    {
        robot_ = Point{x,y};
        try 
        {
            setRobot(getFieldIndex(x,y));
        }
        catch(...){}
    }

    void ArtificialPotentialField::setObstacle(double x, double y)
    {
        obstacles_.push_back({x,y});
        try 
        {
            setObstacle(getFieldIndex(x,y));
        }
        catch(...){}
    }

    void ArtificialPotentialField::setObstacle(const Eigen::Vector2d& vec)
    {
        setObstacle(vec(0), vec(1));
    }

    void ArtificialPotentialField::getAttractionField(potential::Field& field)
    {
        std::vector<potential::FieldGrid>* potential_values;
        potential_values = &values_;
        for(auto& value : (*potential_values))
        {
            value.value = value.attraction;
        }
    }

    void ArtificialPotentialField::getRepulsionField(potential::Field& field)
    {
        std::vector<potential::FieldGrid>* potential_values;
        potential_values = &values_;
        for(auto& value : (*potential_values))
        {
            value.value = value.repulsion;
        }
    }

    Point ArtificialPotentialField::getGoal()
    {
        return goal_;
    }
    
    Point ArtificialPotentialField::getRobot()
    {
        return robot_;
    }
    
    void ArtificialPotentialField::getObstacles(std::vector<Point>& obs)
    {
        obs = obstacles_;
    }

    void ArtificialPotentialField::createPotentialField()   //障害物回避のための斥力場計算
    {
        double weight_attraction_field = weight_attraction_field_;
        double distance_threshold_repulsion_field   = distance_threshold_repulsion_field_;
        double weight_repulsion_field               = weight_repulsion_field_;

        std::vector<potential::FieldGrid>* potential_values;
        potential_values = getValues();
        for(auto& value : (*potential_values))
        {

            double x                    = value.x;
            double y                    = value.y;

            double distance_to_goal     = sqrt(pow(x - goal_.x,2)+pow(y - goal_.y,2));
            double attraction_value     = 0.5 * weight_attraction_field * pow(distance_to_goal, 2);
            value.attraction            = attraction_value;

            // if (distance_to_goal < 0.3) value.states[potential::GridInfo::IS_AROUND_GOAL] = true;



            double repulsion_value              = 0;
            for(auto& coord_obstacle : obstacles_)
            {
                double x_obstacle               = coord_obstacle.x;
                double y_obstacle               = coord_obstacle.y;
                double distance_to_obstacle     = sqrt(pow(x-x_obstacle,2) + pow(y-y_obstacle,2));
                if (distance_to_obstacle        <= distance_threshold_repulsion_field)
                {
                    value.states[potential::GridInfo::IS_REPULSION_FIELD_INSIDE] = true;
                    repulsion_value += 0.5 * weight_repulsion_field * pow(1.0/(distance_to_obstacle + 1e-100) - 1.0/(distance_threshold_repulsion_field + 1e-100), 2);
                }
            }
            value.repulsion                     = repulsion_value;

            value.potential                     = value.attraction + value.repulsion;
            value.value                         = value.potential;
            
        }

        for(auto& value : (*potential_values))
        {

            if (value.states[potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
            {
                bool local_minimum              = true;
                std::vector<size_t> serch_indexes;
                getSquareIndex(serch_indexes, value.row, value.col, 1);
                for (auto idx : serch_indexes)
                {
                    try 
                    {
                        if ((*potential_values)[idx].value < value.value)
                        {
                            local_minimum   = false;
                        }

                        if ((*potential_values)[idx].states[potential::GridInfo::IS_REPULSION_FIELD_INSIDE] == false)
                        {
                            value.states[potential::GridInfo::IS_REPULSION_FIELD_EDGE] = true;
                        }
                    }
                    catch(std::out_of_range& oor) 
                    {
                        continue;
                    }
                }

                if(local_minimum && !value.states[potential::GridInfo::IS_GOAL])
                {
                    value.states[potential::GridInfo::IS_LOCAL_MINIMUM] = true;
                }
            }
        }
    }

}