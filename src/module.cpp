#include <ecto/ecto.hpp>

namespace reconstruction
{
  void
  insert_mesh(const std::string& db_url, const std::string& object_id, const std::string& session_id,
              const std::string& mesh_file, const std::string& surfel_file);
}

ECTO_DEFINE_MODULE(object_recognition_reconstruction)
{
  boost::python::def("insert_mesh", reconstruction::insert_mesh);
}
