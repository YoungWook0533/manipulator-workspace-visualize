#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/* Pinocchio ---------------------------------------------------- */
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

/* FCL ---------------------------------------------------------- */
#include <fcl/fcl.h>

/* TF2 ---------------------------------------------------------- */
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <random>
#include <chrono>

class EESamplerNode : public rclcpp::Node
{
public:
  EESamplerNode() : Node("ee_sampler_node"), gen_(rd_())
  {
    /* ─── parameters ─────────────────────────────────────────── */
    urdf_path_  = declare_parameter("urdf_path",  " ");
    ee_frame_   = declare_parameter("ee_frame",   "l_fr3_link8");
    base_frame_ = declare_parameter("base_frame", "base_link");
    fixed_frame_= declare_parameter("fixed_frame","base_footprint");
    out_file_   = declare_parameter("output_file","DBB.txt");
    N_          = declare_parameter("num_samples",100000);

    /* ─── pinocchio model ────────────────────────────────────── */
    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = pinocchio::Data(model_);
    ee_id_ = model_.getFrameId(ee_frame_);

    /* joint limits (hard-coded) */
    LOWER_ = { -2.7437,-1.7837,-2.9007,-3.0421,-2.8065,-0.5445,-3.0159 };
    UPPER_ = {  2.7437, 1.7837, 2.9007,-0.1518, 2.8065, 4.5169, 3.0159 };
    q_.resize(model_.nq);

    for (pinocchio::JointIndex j=1;j<model_.njoints;++j)
      if (model_.joints[j].nq()==1) joint_names_.push_back(model_.names[j]);

    /* ROS pubs -------------------------------------------------- */
    js_pub_     = create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/collision_markers",10);

    ofs_.open(out_file_, std::ios::out|std::ios::trunc);

    /* TF -------------------------------------------------------- */
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this,[](rclcpp::Node*){});
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_ptr);

    /* FCL + markers -------------------------------------------- */
    createFCLObjects();
    initMarkers();

    /* shell bins (5°×5°) --------------------------------------- */
    initShellBins(1.0);   // 5-degree resolution

    /* timer 1 kHz ---------------------------------------------- */
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1ms, std::bind(&EESamplerNode::step,this));

    RCLCPP_INFO(get_logger(),"Sampling until %d raw samples processed…",N_);
  }

  ~EESamplerNode() override { if(ofs_.is_open()) dumpShell(); }

private:
  /* ───── workspace shell data structure ───── */
  struct Extremes {
    double min_r{-1.0}, max_r{-1.0};
    Eigen::Vector3d min_p, max_p;
  };
  double deg_step_{1.0};
  int az_bins_, el_bins_;
  std::vector<Extremes> shell_;

  void initShellBins(double deg_step)
  {
    deg_step_ = deg_step;
    az_bins_  = int(360 / deg_step_);
    el_bins_  = int(180 / deg_step_);
    shell_.resize(az_bins_ * el_bins_);
  }

  /* ───── main 1 ms callback ───── */
  void step()
  {
    if(samples_ >= N_) { dumpShell(); rclcpp::shutdown(); return; }
    ++samples_;

    /* random joint vector */
    for(int i=0;i<model_.nq;++i)
      q_[i]=std::uniform_real_distribution<double>(LOWER_[i],UPPER_[i])(gen_);

    /* FK */
    pinocchio::forwardKinematics(model_,data_,q_);
    pinocchio::updateFramePlacements(model_,data_);

    /* publish joint state (optional for rsp) */
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.assign(q_.data(), q_.data()+q_.size());
    js_pub_->publish(js);

    /* TF lookup */
    geometry_msgs::msg::TransformStamped tf_base,tf_ee;
    try{
      tf_base=tf_buffer_->lookupTransform(fixed_frame_,base_frame_,rclcpp::Time(0));
      tf_ee  =tf_buffer_->lookupTransform(fixed_frame_,ee_frame_,  rclcpp::Time(0));
    }catch(const tf2::TransformException&){ return; }

    updateFCLTransforms(tf_base,tf_ee);
    publishMarkers(tf_base,tf_ee);

    req_.enable_nearest_points=false; res_.clear();
    if(fcl::distance(base_box1_.get(),ee_sphere_.get(),req_,res_)<=0.0) return;
    res_.clear();
    if(fcl::distance(base_box2_.get(),ee_sphere_.get(),req_,res_)<=0.0) return;
    res_.clear();
    if(fcl::distance(base_box3_.get(),ee_sphere_.get(),req_,res_)<=0.0) return;

    /* ---- shell update ---- */
    Eigen::Vector3d p = tf2::transformToEigen(tf_ee.transform).translation();
    double r = p.norm(); if(r < 1e-6) return;

    double az = std::atan2(p.y(),p.x()) * 180.0/M_PI; if(az<0) az+=360.0;
    double el = std::asin(p.z()/r) * 180.0/M_PI + 90.0;

    int az_i = int(az / deg_step_);
    int el_i = int(el / deg_step_);
    int idx  = el_i * az_bins_ + az_i;

    auto &bin = shell_[idx];
    if(bin.max_r<0 || r>bin.max_r){ bin.max_r=r; bin.max_p=p; }
    if(bin.min_r<0 || r<bin.min_r){ bin.min_r=r; bin.min_p=p; }
    RCLCPP_INFO(this->get_logger(), "Sampled %d / %d", samples_, N_);
  }

  /* ───── dump shell to DBB.txt when done ───── */
  void dumpShell()
  {
    constexpr double MAX=1.185;
    auto to_inten=[](double r){ return std::clamp(std::round(r/MAX*255.0),0.0,255.0); };

    for(const auto &b:shell_)
    {
      if(b.max_r>0){
        ofs_<<b.max_p.x()<<' '<<b.max_p.y()<<' '<<b.max_p.z()<<' '<<to_inten(b.max_r)<<'\n';
      }
      if(b.min_r>0){
        ofs_<<b.min_p.x()<<' '<<b.min_p.y()<<' '<<b.min_p.z()<<' '<<to_inten(b.min_r)<<'\n';
      }
    }
    ofs_.close();
    RCLCPP_INFO(get_logger(),"Workspace shell written to %s (%zu bins).", out_file_.c_str(), shell_.size());
  }

  /* ───── FCL / markers (same as previous version) ───── */
  void createFCLObjects()
  {
    base_box1_=std::make_shared<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(0.9874,0.5709,0.12375));
    base_box2_=std::make_shared<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(0.78992,0.5709,0.10375));
    base_box3_=std::make_shared<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(0.78992,0.4,0.4));
    ee_sphere_=std::make_shared<fcl::CollisionObjectd>(std::make_shared<fcl::Sphered>(0.05));

    offset1_=Eigen::Translation3d(0,0,0.061875)*Eigen::Isometry3d::Identity();
    offset2_=Eigen::Translation3d(0,0,0.175625)*Eigen::Isometry3d::Identity();
    offset3_=Eigen::Translation3d(0,0,0.225)*Eigen::Isometry3d::Identity();
  }

  void initMarkers()
  {
    marker_array_.markers.resize(4);

    /* box1 */
    auto &m0=marker_array_.markers[0];
    m0.header.frame_id=fixed_frame_;
    m0.ns="base_box1"; m0.id=0; m0.type=m0.CUBE; m0.action=m0.ADD;
    m0.scale.x=0.9874; m0.scale.y=0.5709; m0.scale.z=0.12375;
    m0.color.r=1.0; m0.color.g=0.0; m0.color.b=0.0; m0.color.a=0.7;

    /* box2 */
    auto &m1=marker_array_.markers[1];
    m1.header.frame_id=fixed_frame_;
    m1.ns="base_box2"; m1.id=1; m1.type=m1.CUBE; m1.action=m1.ADD;
    m1.scale.x=0.78992; m1.scale.y=0.5709; m1.scale.z=0.10375;
    m1.color.r=1.0; m1.color.g=0.0; m1.color.b=0.0; m1.color.a=0.7;

    auto &m2=marker_array_.markers[2];
    m2.header.frame_id=fixed_frame_;
    m2.ns="base_box3"; m2.id=1; m2.type=m2.CUBE; m2.action=m2.ADD;
    m2.scale.x=0.78992; m2.scale.y=0.4; m2.scale.z=0.4;
    m2.color.r=0.0; m2.color.g=1.0; m2.color.b=0.0; m2.color.a=0.7;

    /* sphere */
    auto &m3=marker_array_.markers[3];
    m3.header.frame_id=fixed_frame_;
    m3.ns="ee_sphere"; m3.id=2; m3.type=m3.SPHERE; m3.action=m3.ADD;
    m3.scale.x=m3.scale.y=m3.scale.z=0.10;   // diameter = 0.1
    m3.color.r=0.0; m3.color.g=1.0; m3.color.b=0.0; m3.color.a=0.8;
  }

  void publishMarkers(const geometry_msgs::msg::TransformStamped& tf_base,
                      const geometry_msgs::msg::TransformStamped& tf_ee)
  {
    /* base boxes ------------------------------------------------- */
    Eigen::Isometry3d T_base = tf2::transformToEigen(tf_base.transform);
    Eigen::Isometry3d T1 = T_base * offset1_;
    Eigen::Isometry3d T2 = T_base * offset2_;
    Eigen::Isometry3d T3 = T_base * offset3_;

    marker_array_.markers[0].header.stamp = now();
    marker_array_.markers[0].pose = tf2::toMsg(T1);      // ✔ 리턴값 사용

    marker_array_.markers[1].header.stamp = now();
    marker_array_.markers[1].pose = tf2::toMsg(T2);      // ✔ 리턴값 사용

    marker_array_.markers[2].header.stamp = now();
    marker_array_.markers[2].pose = tf2::toMsg(T3);      // ✔ 리턴값 사용

    /* EE sphere -------------------------------------------------- */
    marker_array_.markers[3].header.stamp = now();
    // Transform → Isometry → Pose 로 변환
    Eigen::Isometry3d T_ee = tf2::transformToEigen(tf_ee.transform);
    marker_array_.markers[3].pose = tf2::toMsg(T_ee);

    marker_pub_->publish(marker_array_);
  }

  void updateFCLTransforms(const geometry_msgs::msg::TransformStamped& tf_base,
                           const geometry_msgs::msg::TransformStamped& tf_ee)
  {
    Eigen::Isometry3d T_base=tf2::transformToEigen(tf_base.transform);
    Eigen::Isometry3d T_ee  =tf2::transformToEigen(tf_ee.transform);

    base_box1_->setTransform(fcl::Transform3d(T_base*offset1_));
    base_box2_->setTransform(fcl::Transform3d(T_base*offset2_));
    base_box3_->setTransform(fcl::Transform3d(T_base*offset3_));
    ee_sphere_->setTransform(fcl::Transform3d(T_ee));
  }

  /* ─── members ──────────────────────────────────────────────── */
  pinocchio::Model model_; pinocchio::Data data_; pinocchio::FrameIndex ee_id_;
  std::string urdf_path_,ee_frame_,base_frame_,fixed_frame_,out_file_;
  std::vector<double> LOWER_,UPPER_; Eigen::VectorXd q_;
  std::vector<std::string> joint_names_;
  std::ofstream ofs_; int N_{0},samples_{0};

  std::random_device rd_; std::mt19937 gen_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  fcl::DistanceRequestd req_; fcl::DistanceResultd res_;
  std::shared_ptr<fcl::CollisionObjectd> base_box1_,base_box2_,base_box3_,ee_sphere_;
  Eigen::Isometry3d offset1_,offset2_,offset3_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<EESamplerNode>());
  return 0;
}
