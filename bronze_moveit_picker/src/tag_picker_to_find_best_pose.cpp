#include <memory>
#include <chrono>
#include <thread>
#include <map>
#include <algorithm>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene/planning_scene.hpp>  // 추가
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
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
        move_group_interface_->setMaxAccelerationScalingFactor(0.6);
        move_group_interface_->setMaxVelocityScalingFactor(0.9);
        move_group_interface_->setPlanningTime(15.0);  // Set planning time to 15 seconds
        move_group_interface_->setNumPlanningAttempts(100);
        
        // 🔧 STEP 1: CurrentStateMonitor가 /joint_states를 받을 때까지 대기
        move_group_interface_->startStateMonitor();
        RCLCPP_INFO(get_logger(), "Waiting for /joint_states to be received...");
        
        // 간단하게 3초 대기 (API 호환성 문제로 직접 대기)
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(get_logger(), "State monitor initialization completed!");
        
        // Move to home position first
        if (!moveToHome()) {
            RCLCPP_WARN(get_logger(), "Failed to move to home position, continuing anyway...");
        }

        // controlGripper(false);  // Close gripper to ensure it's ready

        // Scan and store all visible AprilTags before starting operations
        if (!scanAndStoreAllTags()) {
            RCLCPP_ERROR(get_logger(), "No AprilTags found! Make sure ttagStandard41h12 detection is running.");
            return false;
        }

            
        // 스캔된 첫 번째 태그 ID 자동 선택
        if (stored_tag_transforms_.empty()) {
            RCLCPP_ERROR(get_logger(), "No tags stored after scanning!");
            return false;
        }
        
        int tag_id = stored_tag_transforms_.begin()->first;  // 첫 번째 태그 ID 선택
        RCLCPP_INFO(get_logger(), "Automatically selected tag ID: %d", tag_id);

        // Pick operation
        if (!executePick(tag_id)) {
            RCLCPP_ERROR(get_logger(), "Pick operation failed for tag ID %d", tag_id);
            return false;
        }
    
        
        RCLCPP_INFO(get_logger(), "Pick until grasp operation successful!");
        return true;
    }

private:
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
    
    // Storage for tag transforms
    std::map<int, geometry_msgs::msg::TransformStamped> stored_tag_transforms_;

    // Constants
    static constexpr double APPROACH_HEIGHT = 0.09;  // 9cm above tag
    static constexpr double CAM_HEIGHT = 0.07;  // 7cm above TCP
    static constexpr double PICK_HEIGHT = -0.02;    // 0 above tag
    static constexpr double LIFT_HEIGHT = 0.10;     // 10cm lift
    static constexpr double PLACE_HEIGHT = 0.02;   // 2cm above tag for placing
    static constexpr double EEF_STEP = 0.005;
    static constexpr double MIN_PATH_FRACTION = 0.6;
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 1500;
    static constexpr int STABILIZE_DELAY_MS = 1000;
    static constexpr int OPERATION_DELAY_MS = 3000;
    static constexpr double MIN_MANIPULABILITY = 0.49; // Minimum manipulability threshold

    bool scanAndStoreAllTags()
    {
        RCLCPP_INFO(get_logger(), "Scanning and storing all visible AprilTags...");
        
        stored_tag_transforms_.clear();
        int tags_found = 0;
        
        // Try to find all visible tags
        for (int tag_id = 0; tag_id < 10; ++tag_id) {
            std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
            
            try {
                geometry_msgs::msg::TransformStamped tag_transform = tf_buffer_->lookupTransform(
                    "base_link",  // target frame
                    tag_frame,  // source frame  
                    tf2::TimePointZero,  // get latest available
                    std::chrono::seconds(1));
                
                // Store the transform
                stored_tag_transforms_[tag_id] = tag_transform;
                tags_found++;
                
                RCLCPP_INFO(get_logger(), "Stored AprilTag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                           tag_id,
                           tag_transform.transform.translation.x,
                           tag_transform.transform.translation.y,
                           tag_transform.transform.translation.z);
            }
            catch (tf2::TransformException &ex) {
                continue;
            }
        }
        
        RCLCPP_INFO(get_logger(), "Found and stored %d AprilTags", tags_found);
        return tags_found > 0;
    }

    bool getStoredTagTransform(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
    {
        auto it = stored_tag_transforms_.find(tag_id);
        if (it != stored_tag_transforms_.end()) {
            tag_transform = it->second;
            RCLCPP_INFO(get_logger(), "Retrieved stored transform for tag ID %d", tag_id);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "No stored transform found for tag ID %d", tag_id);
            return false;
        }
    }

    bool findAprilTag(geometry_msgs::msg::TransformStamped& tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Searching for AprilTag...");
        
        // Try to find any tagStandard41h12 transform
        for (int tag_id = 0; tag_id < 100; ++tag_id) {
            std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
            
            try {
                tag_transform = tf_buffer_->lookupTransform(
                    "base_link",  // target frame
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

    bool moveToHome()
    {
        RCLCPP_INFO(get_logger(), "Moving to home position...");
        
        // Get available named targets for debugging
        std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
        RCLCPP_INFO(get_logger(), "Available named targets:");
        for (const auto& target : named_targets) {
            RCLCPP_INFO(get_logger(), "  - %s", target.c_str());
        }
        
        // Try common home position names
        std::vector<std::string> state_groups = {"home", "ready_to_see", "ready_to_grasp"};

        move_group_interface_->setNamedTarget("ready_to_see");
        
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(home_plan));
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Home plan found! Executing...");
            move_group_interface_->execute(home_plan);
            RCLCPP_INFO(get_logger(), "Successfully moved to home position!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            return true;
        }
        
        RCLCPP_ERROR(get_logger(), "Failed to plan move to home position!");
        return false;
    }

    geometry_msgs::msg::Pose calculateBaseAlignedPose(const geometry_msgs::msg::TransformStamped& tag_transform, double z_offset)
    {
        geometry_msgs::msg::Pose pose;
        
        // Position
        pose.position.x = tag_transform.transform.translation.x;
        pose.position.y = tag_transform.transform.translation.y;
        pose.position.z = tag_transform.transform.translation.z + z_offset;

        // Extract tag's orientation
        tf2::Quaternion tag_quat(
            tag_transform.transform.rotation.x,
            tag_transform.transform.rotation.y,
            tag_transform.transform.rotation.z,
            tag_transform.transform.rotation.w
        );
        
        // Calculate direction vector from tag to robot base (origin)
        double tag_x = tag_transform.transform.translation.x;
        double tag_y = tag_transform.transform.translation.y;
        
        // Direction toward base (normalized)
        double distance_to_base = sqrt(tag_x * tag_x + tag_y * tag_y);
        tf2::Vector3 base_direction(-tag_x / distance_to_base, -tag_y / distance_to_base, 0);
        
        // Get tag's 4 principal axes (+X, -X, +Y, -Y)
        tf2::Vector3 tag_axes[4];
        tag_axes[0] = tf2::quatRotate(tag_quat, tf2::Vector3(1, 0, 0));   // +X  //쿼터니언이랑 벡터 곱함 (벡터는 스칼라 부분(w)에 0을 넣음)
        tag_axes[1] = tf2::quatRotate(tag_quat, tf2::Vector3(-1, 0, 0));  // -X
        tag_axes[2] = tf2::quatRotate(tag_quat, tf2::Vector3(0, 1, 0));   // +Y
        tag_axes[3] = tf2::quatRotate(tag_quat, tf2::Vector3(0, -1, 0));  // -Y
        
        // Project all tag axes to XY plane (remove Z component)
        for (int i = 0; i < 4; i++) {
            tag_axes[i].setZ(0);
            tag_axes[i].normalize();//정규화는 왜 해야하는지 모르겠음
        }
        
        // Find the tag axis that is most aligned with base direction
        double max_dot_product = -1.0;
        int best_axis_index = 0;
        std::string axis_names[4] = {"+X", "-X", "+Y", "-Y"};
        
        for (int i = 0; i < 4; i++) {
            double dot_product = base_direction.dot(tag_axes[i]);
            if (dot_product > max_dot_product) {
                max_dot_product = dot_product;
                best_axis_index = i;
            }
        }
        
        RCLCPP_INFO(get_logger(), "Base direction: (%.3f, %.3f), Best alignment axis: Tag %s (dot product: %.3f)", 
                   base_direction.x(), base_direction.y(), axis_names[best_axis_index].c_str(), max_dot_product);
        
        // Create TCP orientation with base-pointing axis alignment
        tf2::Vector3 tcp_z_axis(0, 0, -1);  // Point down (maintain vertical orientation)
        
        // 🔧 특이점 방지: Z축과 Y축이 거의 평행하면 Z축을 살짝 기울임
        if (fabs(tcp_z_axis.dot(tag_axes[best_axis_index])) > 0.98) {
            // 거의 평행 → 5° 기울여 특이자 탈출
            tcp_z_axis = tf2::quatRotate(tf2::Quaternion(tf2::Vector3(1,0,0), 0.087), tcp_z_axis);
            RCLCPP_WARN(get_logger(), "특이점 방지를 위해 TCP Z축을 5도 기울임");
        }
        
        tf2::Vector3 tcp_y_axis = tag_axes[best_axis_index];  // Use base-pointing tag axis
        tf2::Vector3 tcp_x_axis = tcp_y_axis.cross(tcp_z_axis);
        tcp_x_axis.normalize();
        
        // Recalculate Y-axis to ensure orthogonality
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

    bool updateStoredTagIfVisible(int tag_id)
    {
        // return false;
        RCLCPP_INFO(get_logger(), "Checking if tag ID %d is visible for pose update...", tag_id);
        
        std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
        
        try {
            geometry_msgs::msg::TransformStamped updated_transform = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                tag_frame,  // source frame  
                tf2::TimePointZero,  // get latest available
                std::chrono::seconds(1));
            
            // Update the stored transform with the new, more accurate data
            stored_tag_transforms_[tag_id] = updated_transform;
            
            RCLCPP_INFO(get_logger(), "Updated stored transform for tag ID %d: x=%.3f, y=%.3f, z=%.3f", 
                       tag_id,
                       updated_transform.transform.translation.x,
                       updated_transform.transform.translation.y,
                       updated_transform.transform.translation.z);
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Tag ID %d not visible from current position: %s", tag_id, ex.what());
            return false;
        }
    }

    // Struct for pose with manipulability (currently not used)
    // struct PoseWithManipulability { 
    //     geometry_msgs::msg::Pose pose; 
    //     double manipulability; 
    // };

    // Generate simple pose based on tag transform (no manipulability calculation)
    geometry_msgs::msg::Pose generateTargetPose(const geometry_msgs::msg::TransformStamped& tag_transform)
    {
        RCLCPP_INFO(get_logger(), "Generating target pose for tag transform...");
        geometry_msgs::msg::Pose target_pose = calculateBaseAlignedPose(tag_transform, PICK_HEIGHT);
        return target_pose;
    }
        // double best_manipulability = candidates[0].manipulability;



    // double calculateManipulability(const geometry_msgs::msg::Pose& target_pose)
    // {
    //     try {
    //         // Get current robot state
    //         moveit::core::RobotStatePtr state_ptr = move_group_interface_->getCurrentState();
    //         if (!state_ptr)
    //         {
    //             RCLCPP_ERROR(get_logger(), "Failed to get current RobotState!");
    //             return 0.0;
    //         }
    //         // Create a RobotState object
    //         // 현재 로봇 상태를 가져옴
    //         // getCurrentState()는 현재 로봇 상태를 가져오는 함수
    //         // move_group_interface_는 MoveGroupInterface 객체로, 로봇의 현재 상태

    //         // Get the robot model
    //         const auto& robot_model = move_group_interface_->getRobotModel();

    //         const moveit::core::JointModelGroup* joint_model_group = state_ptr->getJointModelGroup("arm_group");
    //         if (!joint_model_group) {
    //             RCLCPP_ERROR(get_logger(), "JointModelGroup 'arm_group' not found!");
    //             return 0.0;
    //         }   
            
    //         // Try to find IK solution
    //         if (!state_ptr->setFromIK(joint_model_group, target_pose, "TCP")) {
    //             return 0.0;  // No IK solution found
    //         }
            
    //         // Get Jacobian matrix
    //         Eigen::MatrixXd jacobian;
    //         state_ptr->getJacobian(joint_model_group,
    //                                 state_ptr->getLinkModel("TCP"),
    //                                 Eigen::Vector3d::Zero(),
    //                                 jacobian);
            
    //         // Calculate manipulability = sqrt(det(J * J^T))
    //         Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
    //         double manipulability = sqrt(std::abs(jjt.determinant()));
            
    //         return manipulability;
    //     }
    //     catch (const std::exception& e) { // 예외 처리용으로 미리 정의된 에러들을 e로 받아옴
    //         RCLCPP_WARN(get_logger(), "Manipulability calculation failed: %s", e.what()); // 로그 띄움
    //         return 0.0;
    //     }
    // }



    bool moveToApproachPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id)
    {
        (void)tag_id;  // unused parameter 경고 제거
        auto approach_pose = calculateBaseAlignedPose(tag_transform, APPROACH_HEIGHT);
        
        // 이걸 manipulability의 best pose로 바꿔야함
        //  approach_pose 값 계산, 목표자세 설정.

        RCLCPP_INFO(get_logger(), "Moving to approach position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
                   APPROACH_HEIGHT * 100, approach_pose.position.x, 
                   approach_pose.position.y, approach_pose.position.z);

        // 🔧 IK 성공률 향상을 위한 orientation tolerance 설정
        move_group_interface_->setGoalOrientationTolerance(0.17); // 10도 허용

        // ✅ IK 가능 여부 디버깅 - 올바른 네임스페이스 사용
        const moveit::core::JointModelGroup* group = move_group_interface_->getRobotModel()->getJointModelGroup("arm_group");
        moveit::core::RobotStatePtr state(new moveit::core::RobotState(move_group_interface_->getRobotModel()));
        state->setToDefaultValues();

        bool found_ik = state->setFromIK(group, approach_pose, 0.1);

        if (!found_ik) {
            RCLCPP_WARN(get_logger(), "첫 번째 IK 시도 실패, orientation 완화 버전으로 재시도...");
            // orientation 완화 버전 시도 - 현재 pose의 orientation 사용
            geometry_msgs::msg::Pose relaxed_pose = approach_pose;
            relaxed_pose.orientation = move_group_interface_->getCurrentPose("TCP").pose.orientation;
            found_ik = state->setFromIK(group, relaxed_pose, 0.1);
            if (found_ik) {
                approach_pose = relaxed_pose;
                RCLCPP_INFO(get_logger(), "완화된 orientation으로 IK 성공!");
            }
        }

        if (!found_ik) {
            RCLCPP_ERROR(get_logger(), "❌ 모든 IK 시도 실패! 접근 pose에 도달 불가능");
            return false;
        } else {
            RCLCPP_INFO(get_logger(), "✅ IK 성공! 접근 pose reachable");
        }

        // 🔧 STEP 2 & 3: 안전한 시작 상태 설정
        std::vector<double> joint_values;
        state->copyJointGroupPositions(group, joint_values);
        
        if (joint_values.empty()) {
            RCLCPP_WARN(get_logger(), "joint_values가 비어있음, default values로 대체");
            state->setToDefaultValues();  // 파라미터 없이 호출
        } else {
            RCLCPP_INFO(get_logger(), "현재 joint state 사용 (관절 수: %zu)", joint_values.size());
        }
        
        // setFromIK()로 이미 모든 관절값이 설정되었으므로 setVariablePositions() 불필요
        move_group_interface_->setStartState(*state);
        
        // ✅ pose target 설정
        move_group_interface_->setPoseTarget(approach_pose, "TCP");

        // 🔧 Planning 파라미터 대폭 완화
        move_group_interface_->setPlanningTime(15.0);   // 15초로 증가
        move_group_interface_->setNumPlanningAttempts(10); // 재시도 횟수 증가

        // Plan and execute approach motion
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
        //경로 계획 (성공여부)

        // 🔧 Planning 실패시 Cartesian 백업 플랜 시도
        if (!success) {
            RCLCPP_WARN(get_logger(), "일반 planning 실패, Cartesian 경로로 백업 시도...");
            std::vector<geometry_msgs::msg::Pose> waypoints{approach_pose};
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, trajectory);
            
            if (fraction > 0.9) {
                RCLCPP_INFO(get_logger(), "Cartesian 백업 성공! (fraction: %.2f)", fraction);
                move_group_interface_->execute(trajectory);
                success = true;
            } else {
                RCLCPP_ERROR(get_logger(), "Cartesian 백업도 실패 (fraction: %.2f)", fraction);
            }
        }

        if (success) {
            RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
            if (!approach_plan.trajectory.joint_trajectory.points.empty()) {
                move_group_interface_->execute(approach_plan);
            }
            RCLCPP_INFO(get_logger(), "Approach motion completed!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            return true;
        }

        RCLCPP_ERROR(get_logger(), "Failed to plan approach motion!");
        return false;
    }

    bool moveToReacquireTagPosition(const geometry_msgs::msg::TransformStamped& tag_transform, int tag_id = -1)
    {
        // Use look-at pose for approach position to orient TCP toward tag center
        auto tag_pose = calculateBaseAlignedPose(tag_transform, CAM_HEIGHT + APPROACH_HEIGHT);

        RCLCPP_INFO(get_logger(), "Moving to tag position (%.1fcm above) with look-at orientation: x=%.3f, y=%.3f, z=%.3f",
                   CAM_HEIGHT * 100, tag_pose.position.x,
                   tag_pose.position.y, tag_pose.position.z);

        move_group_interface_->setPoseTarget(tag_pose, "jetcocam");

        // Plan and execute approach motion
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        bool success = static_cast<bool>(move_group_interface_->plan(approach_plan));
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Approach plan found! Executing...");
            move_group_interface_->execute(approach_plan);
            RCLCPP_INFO(get_logger(), "Approach motion completed!");
            std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
            
            // Try to update the stored tag pose if the tag is visible from the new position
            if (tag_id >= 0) {
                updateStoredTagIfVisible(tag_id);
            }
            
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
            std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
            //태그 id 찾기
            
            try {
                updated_tag_transform = tf_buffer_->lookupTransform(
                    "base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(1));
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

    // void controlGripper(bool close)
    // {
    //     auto gripper_msg = std_msgs::msg::Bool();
    //     gripper_msg.data = close;
    //     gripper_pub_->publish(gripper_msg);
        
    //     if (close) {
    //         RCLCPP_INFO(get_logger(), "Closing gripper...");
    //         std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_DELAY_MS));
    //     } else {
    //         RCLCPP_INFO(get_logger(), "Opening gripper...");
    //         std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
    //     }
    // }

    bool findSpecificAprilTag(int tag_id, geometry_msgs::msg::TransformStamped& tag_transform)
    {
        std::string tag_frame = "tagStandard41h12:" + std::to_string(tag_id);
        
        try {
            tag_transform = tf_buffer_->lookupTransform(
                "base_link",  // target frame
                tag_frame,  // source frame  
                tf2::TimePointZero,  // get latest available
                std::chrono::seconds(1));
            
            RCLCPP_INFO(get_logger(), "Found specific AprilTag: %s", tag_frame.c_str());
            RCLCPP_INFO(get_logger(), "Tag position: x=%.3f, y=%.3f, z=%.3f", 
                       tag_transform.transform.translation.x,
                       tag_transform.transform.translation.y,
                       tag_transform.transform.translation.z);
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Failed to find AprilTag ID %d: %s", tag_id, ex.what());
            return false;
        }
        }

    bool executePick(int tag_id)
    {
        RCLCPP_INFO(get_logger(), "Starting pick operation for tag ID: %d", tag_id);
        
        // Get the stored tag transform
        geometry_msgs::msg::TransformStamped tag_transform;
        if (!getStoredTagTransform(tag_id, tag_transform)) {
            RCLCPP_ERROR(get_logger(), "Cannot find stored transform for tag ID %d", tag_id);
            return false;
        }

        // Move to tag position first
        if (!moveToReacquireTagPosition(tag_transform, tag_id)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to tag position for tag %d", tag_id);
            return false;
        }

        //Move to approach position
        if (!moveToApproachPosition(tag_transform, tag_id)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to approach position for tag %d", tag_id);
            return false;
        }
        
        // Get the potentially updated tag transform after approach
        geometry_msgs::msg::TransformStamped updated_tag_transform;
        if (getStoredTagTransform(tag_id, updated_tag_transform)) {
            tag_transform = updated_tag_transform;
            RCLCPP_INFO(get_logger(), "Using updated tag transform for final approach");
        }
        
        // Calculate final target pose using (potentially updated) stored transform
        auto final_target_pose = calculateBaseAlignedPose(tag_transform, PICK_HEIGHT);
        
        // Move to final position using Cartesian path
        std::vector<geometry_msgs::msg::Pose> approach_waypoints{final_target_pose};
        if (!executeCartesianPath(approach_waypoints, "final approach to tag")) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Final aligned motion completed!");
        std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_DELAY_MS));
        
        // // Close gripper
        // controlGripper(true);

        // // Lift object
        // auto lift_pose = final_target_pose; // 일단 넘어가
        // lift_pose.position.z += LIFT_HEIGHT;
        
        // std::vector<geometry_msgs::msg::Pose> lift_waypoints{lift_pose}; // 일단 넘어가
        // if (!executeCartesianPath(lift_waypoints, "lifting object")) {
        //     return false;
        // }
        
        // RCLCPP_INFO(get_logger(), "Pick operation completed for tag ID: %d", tag_id);
        // std::this_thread::sleep_for(std::chrono::milliseconds(OPERATION_DELAY_MS));
        
        return true;
    }

 


};

int main(int argc, char* argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create and execute TagPicker 
    auto tag_picker = std::make_shared<TagPicker>();
    // TagPicker class 의 객체 생성 및 .execute() 호출
    bool success = tag_picker->execute();
    
    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;
}