#include <pangolin/pangolin.h>
#include <Eigen/Core>

#include <unistd.h>

void DrawTrajectory(
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>);

int main(int argc, char **argv)
{
  if (argc == 0)
  {
    std::cout << "Please provide the path to poses.txt file as the first "
                 "argument! `pose_viewer cam0_to_world.txt`";
  }

  const std::string trajectory_file = std::string(argv[1]);

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
    poses;
  std::ifstream fin(trajectory_file);
  if (!fin)
  {
    std::cout << "cannot find trajectory file at " << trajectory_file
              << std::endl;
    return 1;
  }

  while (!fin.eof())
  {
    double index, m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34,
      m41, m42, m43, m44;
    fin >> index >> m11 >> m12 >> m13 >> m14 >> m21 >> m22 >> m23 >> m24 >>
      m31 >> m32 >> m33 >> m34 >> m41 >> m42 >> m43 >> m44;

    Eigen::Matrix<double, 3, 3> mat;
    mat << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    Eigen::Isometry3d Twr(mat);
    Twr.pretranslate(Eigen::Vector3d(m14 * 0.001, m24 * 0.001, m34 * 0.001));
    poses.push_back(Twr);
  }
  std::cout << "read total " << poses.size() << " pose entries" << std::endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
    poses)
{
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1280, 720, 500, 500, 512, 389, 0.0001, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  auto handler = std::make_unique<pangolin::Handler3D>(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                            .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 720.0f)
                            .SetHandler(handler.get());

  while (!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    // Draw World coordinate frame
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.5, 0.0, 0.0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.5, 0.0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.5);
    glEnd();

    const auto x_unit_vec = Eigen::Vector3d(1, 0, 0);
    const auto y_unit_vec = Eigen::Vector3d(0, 1, 0);
    const auto z_unit_vec = Eigen::Vector3d(0, 0, 1);

    // Draw axis for each pose
    for (size_t i = 0; i < poses.size(); i++)
    {
      const Eigen::Vector3d Ow = poses[i].translation();
      const Eigen::Vector3d Xw = poses[i] * (0.001 * x_unit_vec);
      const Eigen::Vector3d Yw = poses[i] * (0.001 * y_unit_vec);
      const Eigen::Vector3d Zw = poses[i] * (0.001 * z_unit_vec);
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }

    // Draw trajectory line (between transformations)
    for (size_t i = 0; i < poses.size(); i++)
    {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      const auto p1 = poses[i];
      const auto p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);  // sleep 5 ms
  }
}
