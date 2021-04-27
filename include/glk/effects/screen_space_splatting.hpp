#ifndef GLK_SCREEN_SPACE_SPLATTING_HPP
#define GLK_SCREEN_SPACE_SPLATTING_HPP

#include <glk/query.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/transform_feedback.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class Profiler;

class ScreenSpaceSplatting : public ScreenEffect {
public:
  ScreenSpaceSplatting(const Eigen::Vector2i& size = Eigen::Vector2i(1920, 1080));
  virtual ~ScreenSpaceSplatting() override;

  virtual void set_size(const Eigen::Vector2i& size) override;
  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  void extract_points_on_screen(const glk::Texture& depth_texture);
  void estimate_initial_radius(const glk::Texture& depth_texture);
  void estimate_knn_radius();
  void estimate_gaussian(Profiler& prof, const TextureRenderer& renderer);

private:
  glk::GLSLShader texture_shader;
  glk::GLSLShader position_shader;
  std::unique_ptr<glk::FrameBuffer> position_buffer;

  glk::GLSLShader white_shader;
  glk::GLSLShader point_extraction_shader;
  std::unique_ptr<glk::PointCloudBuffer> sampling_points_buffer;
  std::unique_ptr<glk::TransformFeedback> points_on_screen;
  std::unique_ptr<glk::Query> query;

  glk::GLSLShader debug_shader;

  // initial radius estimation
  int k_neighbors;
  int k_tolerance;
  int initial_estimation_grid_size;
  glk::GLSLShader increment_shader;
  std::unique_ptr<glk::FrameBuffer> initial_estimation_buffer;

  // knn estimation
  int num_iterations;
  glk::GLSLShader initial_radius_shader;
  glk::GLSLShader distribution_shader;
  glk::GLSLShader gathering_shader;
  glk::GLSLShader bounds_update_shader;

  std::unique_ptr<glk::FrameBuffer> radius_buffer_ping;
  std::unique_ptr<glk::FrameBuffer> radius_buffer_pong;
  std::unique_ptr<glk::FrameBuffer> neighbor_counts_buffer;
  std::unique_ptr<glk::FrameBuffer> feedback_radius_buffer;

  // computed radius
  glk::GLSLShader farthest_point_shader;
  glk::GLSLShader radius_finalization_shader;
  std::unique_ptr<glk::FrameBuffer> finalized_radius_buffer;

  // gaussian estimation
  glk::GLSLShader gaussian_gathering_shader;
  glk::GLSLShader gaussian_finalization_shader;
  std::unique_ptr<glk::FrameBuffer> gaussian_accum_buffer;
  std::unique_ptr<glk::FrameBuffer> gaussian_dists_buffer;
};

}  // namespace glk

#endif