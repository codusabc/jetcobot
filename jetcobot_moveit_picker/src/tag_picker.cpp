#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>

// ROS2 and MoveIt headers 
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

class TagPicker : public rclcpp::Node // Inherit from rclcpp::Node to use ROS features
{
public:
    TagPicker() : Node("tag_picker"), // 노드 이름 지정
                  tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())), // 멤버변수를 초기화 리스트에서 초기화 (unique: 소유권 단독)
                  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) // shared_ptr 여러곳과 공유 가능하게 초기화
    {
        // Create gripper command publisher
        gripper_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_command", 10);
        // /gripper_command 토픽에 메시지를 퍼블리시할 퍼블리셔 생성
        
        RCLCPP_INFO(get_logger(), "TagPicker initialized successfully");
    }

    bool execute()
    {
        // Initialize MoveGroupInterface after the object is fully constructed
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        move_group_interface_->setMaxVelocityScalingFactor(1.0);
        move_group_interface_->setPlanningTime(15.0);  // Set planning time to 15 seconds
        move_group_interface_->setNumPlanningAttempts(30);
        
        // Find AprilTag
        geometry_msgs::msg::TransformStamped tag_transform;
        if (!findAprilTag(tag_transform)) {
            RCLCPP_ERROR(get_logger(), "No AprilTag found! Make sure tag36h11 detection is running.");
            return false;
        }

        // Move to approach position
        if (!moveToApproachPosition(tag_transform)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to approach position");
            return false;
        }

        // Re-acquire tag transform for better accuracy
        geometry_msgs::msg::TransformStamped updated_tag_transform;
        if (!reacquireTagTransform(updated_tag_transform)) {
            RCLCPP_WARN(get_logger(), "Lost AprilTag after approach! Using original transform.");
            updated_tag_transform = tag_transform;
        }

        // Execute pick and place sequence
        return executePickAndPlace(updated_tag_transform);
    }

private:
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;

    // Constants
    static constexpr double APPROACH_HEIGHT = 0.1;  // 10cm above tag
    static constexpr double PICK_HEIGHT = -0.01;    // 1cm above tag
    static constexpr double LIFT_HEIGHT = 0.10;     // 6cm lift
    static constexpr double EEF_STEP = 0.01;
    static constexpr double MIN_PATH_FRACTION = 0.8;
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 3000;
    static constexpr int STABILIZE_DELAY_MS = 500;
    static constexpr int OPERATION_DELAY_MS = 1000;
    
    // Manipulability constants
    static constexpr double MIN_MANIPULABILITY = 0.049;

    // Function to find AprilTag 
    bool findAprilTag(geometry_msgs::msg::TransformStamped& tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Searching for AprilTag...");
        
        // Try to find any tag36h11 transform
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            
            try {
                tag_transform = tf_buffer_->lookupTransform(
                    "link1",  // target frame
                    tag_frame,  // source frame  
                    tf2::TimePointZero,  // get latest available
                    std::chrono::seconds(1));
                
                RCLCPP_INFO(get_logger(), "Found AprilTag: %s", tag_frame.c_str());
                RCLCPP_INFO(get_logger(), "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                           tag_transform.transform.translation.x,
                           tag_transform.transform.translation.y,
                           tag_transform.transform.translation.z);
                return true;
            }
            catch (tf2::TransformException &ex) {
                continue;
            }
        }
        return false;
    }

    geometry_msgs::msg::Pose calculateAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
    {
        geometry_msgs::msg::Pose pose;
        
        // Position
        pose.position.x = tag_transform.transform.translation.x;
        pose.position.y = tag_transform.transform.translation.y;
        pose.position.z = tag_transform.transform.translation.z + z_offset;

        // Extract tag's orientation for Y-axis alignment
        tf2::Quaternion tag_quat(
            tag_transform.transform.rotation.x,
            tag_transform.transform.rotation.y,
            tag_transform.transform.rotation.z,
            tag_transform.transform.rotation.w
        );
        
        // Get tag's Y-axis direction
        tf2::Vector3 tag_y_axis(0, 1, 0); //벡터
        tag_y_axis = tf2::quatRotate(tag_quat, tag_y_axis); 
        //쿼터니언이랑 벡터 곱함 (벡터는 스칼라 부분(w)에 0을 넣음)
        // tag_y_axis는 월드 좌표계에서 실제 태그의 Y축 방향
        
        // Create TCP orientation aligned with tag's Y-axis
        tf2::Vector3 tcp_z_axis(0, 0, -1);  // 그리퍼 숙여
        tf2::Vector3 tcp_y_axis = tag_y_axis;  
        // Y축을 태그의 Y축과 정렬
        // 태그의 Y축과 정렬된 TCP의 Y축
        tf2::Vector3 tcp_x_axis = tcp_y_axis.cross(tcp_z_axis); //cross는 외적임. y랑 z의 외적하면 x축 나옴
        tcp_x_axis.normalize(); //정규화는 왜 해야하는지 모르겠음
        tcp_y_axis = tcp_z_axis.cross(tcp_x_axis);
        tcp_y_axis.normalize();
        
        // Create rotation matrix and convert to quaternion
        tf2::Matrix3x3 rotation_matrix(
            tcp_x_axis.x(), tcp_y_axis.x(), tcp_z_axis.x(),
            tcp_x_axis.y(), tcp_y_axis.y(), tcp_z_axis.y(),
            tcp_x_axis.z(), tcp_y_axis.z(), tcp_z_axis.z()
        );
        
        tf2::Quaternion tcp_quat;
        rotation_matrix.getRotation(tcp_quat);
        tcp_quat.normalize();
        
        pose.orientation.x = tcp_quat.x();
        pose.orientation.y = tcp_quat.y();
        pose.orientation.z = tcp_quat.z();
        pose.orientation.w = tcp_quat.w();
        
        return pose;
    }

    bool moveToApproachPosition(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        auto approach_pose = calculateAlignedPose(tag_transform, APPROACH_HEIGHT);
        //  approach_pose 값 계산, 목표자세 설정.

        RCLCPP_INFO(get_logger(), "Moving to approach position (%.1fcm above): x=%.3f, y=%.3f, z=%.3f", 
                   APPROACH_HEIGHT * 100, approach_pose.position.x, 
                   approach_pose.position.y, approach_pose.position.z);

        move_group_interface_->setPoseTarget(approach_pose, "TCP");
        // Set target pose for the end effector
        // 목표자세를 TCP로 지정?

        // Plan and execute approach motion
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
        //경로 계획 (성공여부)

        if (success) {
            RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
            move_group_interface_->execute(approach_plan);
            // 성공하면 실행
            RCLCPP_INFO(get_logger(), "Approach motion completed!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            return true;
        }
        
        return false;
    }

    bool reacquireTagTransform(geometry_msgs::msg::TransformStamped& updated_tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Re-acquiring AprilTag transform from approach position...");
        // tag 다시 찾아 위치 정확도 보완, 실패하면 원래 값을 그대로 사용

        // Try to find the same tag again
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
            //태그 id 찾기
            
            try {
                updated_tag_transform = tf_buffer_->lookupTransform(
                    "link1", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
                // lookupTransform: link1에서 tag_frame이 어디있는지 tf 변환값 찾아줌
                // tf2::TimePointZero: 현재 시간 기준으로 변환
                // std::chrono::seconds(1): 1초 동안 찾고 못 찾으면 timeout
                RCLCPP_INFO(get_logger(), "Updated AprilTag: %s", tag_frame.c_str());
                RCLCPP_INFO(get_logger(), "Updated tag position: x=%.3f, y=%.3f, z=%.3f", 
                           updated_tag_transform.transform.translation.x,
                           updated_tag_transform.transform.translation.y,
                           updated_tag_transform.transform.translation.z);
                return true;
            }
            catch (tf2::TransformException &ex) { // 위에서 lookupTransform 실패하면 예외 발생
                // &ex: 예외 객체를 참조로 받는 변수 이름, &는 참조 연산자인데 복사하지 않고 예외객체를 직접 다루기 위해 씀
                continue; // 다음 루프로 넘어감? (다시시도)
            }
        }
        return false;
    }

    // Struct for pose with manipulability
    struct PoseWithManipulability { //구조체
        geometry_msgs::msg::Pose pose; // end effector의 pose
        double manipulability; // manipulability 값
    };

    double calculateManipulability(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            // Get current robot state
            moveit::core::RobotStatePtr state_ptr = move_group_interface_->getCurrentState();
            if (!state_ptr)
            {
                RCLCPP_ERROR(get_logger(), "Failed to get current RobotState!");
                return 0.0;
            }
            // Create a RobotState object
            // 현재 로봇 상태를 가져옴
            // getCurrentState()는 현재 로봇 상태를 가져오는 함수
            // move_group_interface_는 MoveGroupInterface 객체로, 로봇의 현재 상태

            // Get the robot model
            const auto& robot_model = move_group_interface_->getRobotModel();

            const moveit::core::JointModelGroup* joint_model_group = state_ptr->getJointModelGroup("arm");
            if (!joint_model_group) {
                RCLCPP_ERROR(get_logger(), "JointModelGroup 'arm' not found!");
                return 0.0;
            }   
            
            // Try to find IK solution
            if (!state_ptr->setFromIK(joint_model_group, target_pose, "TCP")) {
                return 0.0;  // No IK solution found
            }
            
            // Get Jacobian matrix
            Eigen::MatrixXd jacobian;
            state_ptr->getJacobian(joint_model_group,
                                    state_ptr->getLinkModel("TCP"),
                                    Eigen::Vector3d::Zero(),
                                    jacobian);
            
            // Calculate manipulability = sqrt(det(J * J^T))
            Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
            double manipulability = sqrt(std::abs(jjt.determinant()));
            
            return manipulability;
        }
        catch (const std::exception& e) { // 예외 처리용으로 미리 정의된 에러들을 e로 받아옴
            RCLCPP_WARN(get_logger(), "Manipulability calculation failed: %s", e.what()); // 로그 띄움
            return 0.0;
        }
    }

    std::vector<PoseWithManipulability> generateCandidatePoses(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        std::vector<PoseWithManipulability> candidates;
        

        // TODO: 여기에 여러 각도에서 후보 pose들을 생성하는 코드를 작성하세요
        // 예시: 다양한 접근 각도, 회전, 높이 등을 시도
        
        // 기본 pose만 추가 (예시)
        auto base_pose = calculateAlignedPose(tag_transform, PICK_HEIGHT);
        double manip = calculateManipulability(base_pose);
        
        if (manip > MIN_MANIPULABILITY) {
            candidates.push_back({base_pose, manip});
            RCLCPP_INFO(get_logger(), "Base pose manipulability: %.4f", manip);
        }
        
        // Sort by manipulability (highest first)
        std::sort(candidates.begin(), candidates.end(), 
                  [](const auto& a, const auto& b) { return a.manipulability > b.manipulability; });
        
        return candidates;
    }

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& description)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, EEF_STEP, trajectory);
        
        if (fraction > MIN_PATH_FRACTION) {
            RCLCPP_INFO(get_logger(), "%s (path fraction: %.2f)", description.c_str(), fraction);
            move_group_interface_->execute(trajectory);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan %s! Cartesian path fraction: %.2f", description.c_str(), fraction);
            return false;
        }
    }

    void controlGripper(bool close)
    {
        auto gripper_msg = std_msgs::msg::Bool();
        gripper_msg.data = close;
        gripper_pub_->publish(gripper_msg);
        
        if (close) {
            RCLCPP_INFO(get_logger(), "Closing gripper...");
            std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_DELAY_MS));
        } else {
            RCLCPP_INFO(get_logger(), "Opening gripper...");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        }
    }

    bool executePickAndPlace(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        // Generate candidate poses and find the best one based on manipulability
        auto candidates = generateCandidatePoses(tag_transform);
        
        if (candidates.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid poses with good manipulability found!");
            return false;
        }
        
        // Use the pose with the highest manipulability
        auto best_pose = candidates[0].pose;
        RCLCPP_INFO(get_logger(), "Selected pose with manipulability: %.4f", candidates[0].manipulability);
        
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{best_pose};
        if (!executeCartesianPath(approach_waypoints, "final approach to tag")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Final aligned motion completed!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // Close gripper
        controlGripper(true);

        // Lift object
        auto lift_pose = best_pose; // 일단 넘어가
        lift_pose.position.z += LIFT_HEIGHT;
        
        std::vector<geometry_msgs::msg::Pose> lift_waypoints{lift_pose}; // 일단 넘어가
        if (!executeCartesianPath(lift_waypoints, "lifting object")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Object lifted successfully!");
        std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        
        // Move back down to original position
        std::vector<geometry_msgs::msg::Pose> down_waypoints{best_pose}; //일단 넘어가
        if (!executeCartesianPath(down_waypoints, "returning to original position")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Returned to original position!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // Open gripper
        controlGripper(false);
        
        RCLCPP_INFO(get_logger(), "Pick and place operation completed successfully!");
        return true;
    }
};

int main(int argc, char* argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create and execute TagPicker 
    // TagPicker class 의 객체 생성 및 .execute() 호출
    auto tag_picker = std::make_shared<TagPicker>();
    
    bool success = tag_picker->execute();
    
    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;


};

