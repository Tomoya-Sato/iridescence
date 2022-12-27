#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <implot.h>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->set_max_text_buffer_size(1);

  std::vector<double> stamp;
  std::vector<double> data;
  for (int i = 0; i < 100; i++)
  {
    stamp.push_back(i * 0.1);
    data.push_back(i % 10 * 1.0);
  }
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  viewer->register_ui_callback("ui", [&]() {
    ImGui::DragFloat("X", &x, 0.01f);
    ImGui::DragFloat("Y", &y, 0.01f);
    ImGui::DragFloat("Z", &z, 0.01f);
    ImGui::DragFloat("Yaw", &yaw, 0.01f);
    ImGui::DragFloat("Pitch", &pitch, 0.01f);
    ImGui::DragFloat("Roll", &roll, 0.01f);

    if (ImGui::Button("Close")) {
      viewer->close();
    }
    if (ImGui::Button("Print")) {
      std::string text = "X: " + std::to_string(x) + ", Y:" + std::to_string(y);
      viewer->append_text(text);
    }
    ImGui::Begin("Test", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImPlot::SetNextAxisLimits(0, stamp.front(), stamp.back(), ImGuiCond_Always);
    ImPlot::SetNextAxisLimits(3, 0, 15, ImGuiCond_Always);
    if (ImPlot::BeginPlot("test", ImVec2(400, 150))) {
      ImPlot::PlotLine("data", stamp.data(), data.data(), stamp.size());
      ImPlot::EndPlot();
    }
    ImGui::End();
  });

  while (viewer->spin_once()) {

    Eigen::AngleAxisf yaw_rot(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_rot(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf roll_rot(roll, Eigen::Vector3f::UnitX());
    Eigen::Translation<float, 3> trans(x, y, z);
    Eigen::Matrix4f mat = (trans * (roll_rot * pitch_rot * yaw_rot)).matrix();
    std::string text = "X: " + std::to_string(x) + ", Y:" + std::to_string(y);
    viewer->append_text(text);
    for (int i = 0; i < 100; i++)
    {
      data[i] = static_cast<int>(i + x) % 10 * 1.0;
      stamp[i] = (i + y) * 0.1;
    }
    if (x > 10)
      viewer->update_drawable("cube", glk::Primitives::coordinate_system(), guik::FlatRed(mat));
    else
      viewer->update_drawable("cube", glk::Primitives::coordinate_system(), guik::FlatGreen(mat));

  }

  return 0;
}