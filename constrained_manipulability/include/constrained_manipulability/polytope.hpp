
#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <jsk_recognition_msgs/msg/polygon_array.hpp>

namespace constrained_manipulability
{
enum SLICING_PLANE
{
    YZ_PLANE = 0,
    XZ_PLANE = 1,
    XY_PLANE = 2,
};

class Polytope
{
    public:
        Polytope();
        Polytope(const std::string& name, const Eigen::MatrixXd& vertex_set, const Eigen::Vector3d& offset);
        Polytope(const std::string& name, const Eigen::MatrixXd& A_left, const Eigen::VectorXd& b_left, const Eigen::Vector3d& offset);
        //Polytope(const std::string& name, const jsk_recognition_msgs::msg::PolygonArray& jskmesh, const Eigen::MatrixXd& A_left, const Eigen::VectorXd& b_left, const Eigen::Vector3d& offset);
        ~Polytope() {}

        inline bool isValid() const { return (name_ != "invalid_polytope"); }
        inline std::string getName() const { return name_; }
        inline double getVolume() const { return volume_; }
        inline shape_msgs::msg::Mesh getMesh() const { return mesh_; }

        // Converting to jsk visualization mesh - jsk
        inline jsk_recognition_msgs::msg::PolygonArray getjskMesh() const { return jskmesh_; }
        inline std::vector<geometry_msgs::msg::Point> getPoints() const { return points_; }
        //inline std::vector<geometry_msgs::msg::Point> getPoints() const { return jskpoints_; }
        
        // Transform polytope to Cartesian space (offset provided to recompute mesh)
        void transformCartesian(const Eigen::Matrix<double, 3, Eigen::Dynamic>& Jp);

        // yz plane = 0
        // xz plane = 1
        // xy plane = 2
        Polytope slice(const std::string& name,
                       SLICING_PLANE index = SLICING_PLANE::XY_PLANE,
                       double plane_width = 0.002) const;

        // Function to get the intersection of two polytopes
        Polytope getPolytopeIntersection(const std::string& name,
                                         const Eigen::MatrixXd& AHrep_other,
                                         const Eigen::VectorXd& bHrep_other) const;
        // Funciton to get polyotpe array for plotting using jsk-visualization mesh
        // bool getPolytopeArray(const Eigen::Vector3d& offset_position,
        //             std::vector<geometry_msgs::msg::Point>& points,
        //             jsk_recognition_msgs::msg::PolygonArray& jskmesh,
        //             std::string& name) const;

    private:
        // Compute the volume of a polytope defined by its vertex set
        double computeVolume() const;

        /** Construct the Polytope's corresponding mesh using its vertices
         *  These vertices are shifted to the offset position, e.g., the robot end effector
         *  PCL is used to obtain the convex hull
         *  A shape_msgs::Mesh message is created from the convex hull
         *  The return value is a flag for whether the computation was successful or not
         */
        bool constructMesh();
        bool constructjskMesh();

        

        // Polytope properties
        std::string name_;
        double volume_;
        shape_msgs::msg::Mesh mesh_;
        // jsk-recoginitino version of the mesh
        jsk_recognition_msgs::msg::PolygonArray jskmesh_;
        
        std::vector<geometry_msgs::msg::Point> points_;
        geometry_msgs::msg::Point32 pp2_;

        Eigen::MatrixXd vertex_set_;
        Eigen::MatrixXd AHrep_;
        Eigen::VectorXd bHrep_;
        Eigen::VectorXd offset_position_;
};
} // namespace constrained_manipulability