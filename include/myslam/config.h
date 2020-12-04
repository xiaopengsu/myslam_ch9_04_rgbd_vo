/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_;
                //TODO:SXP static std::shared_ptr<Class> example_      https://www.cnblogs.com/xudong-bupt/p/9027609.html
                //        typedef shared_ptr<VisualOdometry> Ptr;
                /*
                num_of_features_    = Config::get<int> ( "number_of_features" );
                scale_factor_       = Config::get<double> ( "scale_factor" );
                level_pyramid_      = Config::get<int> ( "level_pyramid" );
                match_ratio_        = Config::get<float> ( "match_ratio" );
                max_num_lost_       = Config::get<float> ( "max_num_lost" );
                min_inliers_        = Config::get<int> ( "min_inliers" );
                key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
                key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
                map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
                 */
    cv::FileStorage file_;
    
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); 
    
    // access the parameter values
    template< typename T >
    static T get( const std::string& key ) //TODO:SXP  C++中模板template <typename T>  https://www.cnblogs.com/Yu-FeiFei/p/6810221.html
    {
        return T( Config::config_->file_[key] );
    }
};
}

#endif // CONFIG_H
