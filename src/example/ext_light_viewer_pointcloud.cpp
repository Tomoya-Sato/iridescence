#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <portable-file-dialogs.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>

std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> getColorVec(const std::vector<float>& intensity_vec, const float& intensity_range, const float& alpha)
{
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> color_vec(intensity_vec.size());
  for (int i = 0; i < intensity_vec.size(); i++)
  {
    float g = intensity_vec[i] / intensity_range;
    color_vec[i] = Eigen::Vector4f(1.0f, g, 0.0f, alpha);
  }
  return color_vec;
}

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  viewer->enable_info_buffer();

  std::vector<std::string> filenames;
  std::vector<float> intensity_vec;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;
  int intensity_range = 255;
  float alpha = 1.0;
  bool dynamic_rendering = false;
  float scale = 1.0;

  viewer->register_ui_callback("cloud_loader", [&]() {
    ImGui::Checkbox("Dynamic Rendering", &dynamic_rendering);
    if(ImGui::Button("load")) {
      std::vector<std::string> results = pfd::open_file("choose PCD file").result();
      if(!results.empty()) {
        filenames.push_back(results[0]);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(results[0], *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        xyz_cloud->resize(cloud->size());
        intensity_vec.resize(cloud->size());
        for (int i = 0; i < cloud->size(); i++)
        {
          xyz_cloud->points[i].getVector4fMap() = cloud->points[i].getVector4fMap();
          intensity_vec[i] = cloud->points[i].intensity;
        }

        cloud_buffer = glk::create_point_cloud_buffer(*xyz_cloud);
        cloud_buffer->add_color(getColorVec(intensity_vec, intensity_range, alpha)[0].data(), sizeof(Eigen::Vector4f), cloud->size());
        viewer->update_drawable(results[0], cloud_buffer, guik::VertexColor().add("point_scale", scale));

        // *** example usage ***
        // construct PointCloudBuffer from raw float pointer
        // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(cloud->front().data, sizeof(pcl::PointXYZ), cloud->size());

        // use rainbow color (height encoding)
        // viewer->update_drawable(results[0], cloud_buffer, guik::Rainbow());

        // with some transformation and bigger points
        // viewer->update_drawable(results[0], cloud_buffer, guik::FlatColor(Eigen::Vector4f::Random(), Eigen::Translation3f(1.0f, 1.0f, 1.0f)).add("point_scale", 5.0f));

        // with point color
        // std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors(cloud->size());
        // for(int i = 0; i < cloud->size(); i++) {
        //   colors[i] = cloud->at(i).getVector4fMap() * 0.1f;
        // }
        // cloud_buffer->add_color(colors[0].data(), sizeof(Eigen::Vector4f), colors.size());
        // viewer->update_drawable(results[0], cloud_buffer, guik::VertexColor());

        // another way to create colored points with pcl::PointXYZRGBA
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // colored = ...
        // auto cloud_buffer = glk::create_colored_point_cloud_buffer(*colored);
        // viewer->update_drawable(results[0], cloud_buffer, guik::VertexColor());
      }
    }

    if (ImGui::DragFloat("SCALE", &scale, 0.01f, 0.1f, 3.0f) && dynamic_rendering)
    {
      if (!filenames.empty())
      {
        cloud_buffer->add_color(getColorVec(intensity_vec, intensity_range, alpha)[0].data(), sizeof(Eigen::Vector4f), intensity_vec.size());
        viewer->update_drawable(filenames[0], cloud_buffer, guik::VertexColor().add("point_scale", scale));
      }
    }

    if (ImGui::DragFloat("ALPHA", &alpha, 0.1f, -100.0f, 100.0f) && dynamic_rendering)
    {
      if (!filenames.empty())
      {
        cloud_buffer->add_color(getColorVec(intensity_vec, intensity_range, alpha)[0].data(), sizeof(Eigen::Vector4f), intensity_vec.size());
        viewer->update_drawable(filenames[0], cloud_buffer, guik::VertexColor().add("point_scale", scale));
      }
    }

    if (ImGui::DragInt("INTENSITY_RANGE", &intensity_range, 1, 0, 255) && dynamic_rendering)
    {
      if (!filenames.empty())
      {
        cloud_buffer->add_color(getColorVec(intensity_vec, intensity_range, alpha)[0].data(), sizeof(Eigen::Vector4f), intensity_vec.size());
        viewer->update_drawable(filenames[0], cloud_buffer, guik::VertexColor().add("point_scale", scale));
      }
    }

    if (ImGui::Button("Apply"))
    {
      if (!filenames.empty())
      {
        cloud_buffer->add_color(getColorVec(intensity_vec, intensity_range, alpha)[0].data(), sizeof(Eigen::Vector4f), intensity_vec.size());
        viewer->update_drawable(filenames[0], cloud_buffer, guik::VertexColor().add("point_scale", scale));
      }
    }

    for(int i = 0; i < filenames.size(); i++) {
      std::string button_name = "remove##" + std::to_string(i);
      std::string filename = filenames[i];
      if(ImGui::Button(button_name.c_str())) {
        viewer->remove_drawable(filenames[i]);
        filenames.erase(filenames.begin() + i);
      }

      ImGui::SameLine();
      ImGui::Text("%s", filename.c_str());
    }
  });

  viewer->register_ui_callback("picking", [&]() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
      return;
    }

    auto mouse_pos = ImGui::GetMousePos();
    if (ImGui::IsMouseClicked(1)) {
      Eigen::Vector4i info = viewer->pick_info(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
      std::cout << "info: " << info.transpose() << std::endl;

      // if (info[3] == -1) {
      //   return;
      // }

      // get the depth
      float depth = viewer->pick_depth(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
      Eigen::Vector3f pos = viewer->unproject(Eigen::Vector2i(mouse_pos.x, mouse_pos.y), depth);
      std::cout << "depth: " << depth << ", pos: " << pos.transpose() << std::endl;
    }
  });

  viewer->spin();

  return 0;
}