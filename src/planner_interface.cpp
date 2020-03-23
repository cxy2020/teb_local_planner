#include "teb_local_planner/planner_interface.h"
#include "teb_local_planner/timed_elastic_band.h"

namespace teb_local_planner {

int PlannerInterface::GetFirstMappedPoint(const std::vector<geometry_msgs::PoseStamped>& global_path,
                                          const TimedElasticBand& teb,
                                          bool is_moving_forward,
                                          double max_lookahead_length) const
{
    if(teb.sizePoses() == 0) {
        return 0;
    }

    double x = teb.Pose(0).x();
    double y = teb.Pose(0).y();
    double theta = teb.Pose(0).theta();

    int teb_pose_size = teb.sizePoses();
    int teb_index = 0;
    double d_teb_length = 0.0;
    double d_teb_theta = 0.0;
    double max_d_teb_length = 0.0;
    double max_d_teb_theta = 0.0;
    double teb_ratio = 0.0;
    bool is_rotating = false;
    //If the curvature is big, make lookahead length little
    while(++teb_index < teb_pose_size) {
        d_teb_length = abs(teb.Pose(teb_index).x() - x) + abs(teb.Pose(teb_index).y() - y);
        d_teb_theta = abs(teb.Pose(teb_index).theta() - theta);
        if(max_d_teb_length < d_teb_length) {
            max_d_teb_length = d_teb_length;
        }
        if(max_d_teb_theta < d_teb_theta) {
            max_d_teb_theta = d_teb_theta;
        }

        if(max_d_teb_length <= 0.0) {
            max_lookahead_length = 0.0;
            is_rotating = true;
            break;
        }
        teb_ratio = max_d_teb_theta / max_d_teb_length;
        if(teb_ratio > 10.0) {
            max_lookahead_length = max_d_teb_length;
            is_rotating = true;
            break;
        }
        if(teb_ratio < 1.0) {
            break;
        }
    }

    int i = 0;
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    if(!is_moving_forward) {
        sin_theta = -sin_theta;
        cos_theta = -cos_theta;
    }

    double dx = 0.0;
    double dy = 0.0;
    double product = 0.0;

    int g_path_size = global_path.size();
    int max_g_point_index = g_path_size - 1;

    double g_dir_x = 0.0;
    double g_dir_y = 0.0;
    bool is_exist_farther_g_point = false;
    int farther_g_point_index = 0;
    int last_check_index = 0;
    double g_length = 0.0;
    while(i < g_path_size) {
        dx = x - global_path[i].pose.position.x;
        dy = y - global_path[i].pose.position.y;

        product = dx * cos_theta + dy * sin_theta;
        if(product > 0.0) {
            //The last point can be deleted directly
            if(i == max_g_point_index) {
                ++i;
                break;
            }
            //Check the direction of the point on the global path. If the direction consistents with
            //the first point on the teb poses, it is passed by already and can be deleted
            g_dir_x = global_path[i + 1].pose.position.x - global_path[i].pose.position.x;
            g_dir_y = global_path[i + 1].pose.position.y - global_path[i].pose.position.y;

            //when the robot is rotating, only check the global length in max_lookahead_length range
            if(is_rotating) {
                g_length += abs(g_dir_x) + abs(g_dir_y);
                if(g_length > max_lookahead_length) {
                    break;
                }
            }
            product = g_dir_x * cos_theta + g_dir_y * sin_theta;
            if(product >= 0.0) {
                ++i;
                continue;
            }

            //If the direction of the global point doesn't consistents with the local point, check if
            //it exists a point in kMaxCheckDistance that is farther than the global point. If yes,
            //the global point can be deleted
            if(is_exist_farther_g_point) {
                if(i <= farther_g_point_index) {
                    ++i;
                    continue;
                }
                is_exist_farther_g_point = false;
            }
            if(!is_exist_farther_g_point) {
                int check_index = i + 1;
                double check_manhattan_distance = 0.0;
                do {
                    if(check_index >= last_check_index) {
                        dx = x - global_path[check_index].pose.position.x;
                        dy = y - global_path[check_index].pose.position.y;

                        product = dx * cos_theta + dy * sin_theta;
                        if(product <= 0.0) {
                            is_exist_farther_g_point = true;
                            farther_g_point_index = check_index;
                            break;
                        }
                    }

                    g_dir_x = global_path[check_index].pose.position.x - global_path[check_index - 1].pose.position.x;
                    g_dir_y = global_path[check_index].pose.position.y - global_path[check_index - 1].pose.position.y;
                    check_manhattan_distance += (abs(g_dir_x) + abs(g_dir_y));
                    if(check_manhattan_distance > max_lookahead_length) {
                        break;
                    }
                    ++check_index;
                } while(check_index < g_path_size);
                last_check_index = check_index;

                if(is_exist_farther_g_point) {
                    ++i;
                    continue;
                }
                else {
                    break;
                }
            }
        }
        else {
            break;
        }
    }
    return i;
}

} // namespace teb_local_planner
