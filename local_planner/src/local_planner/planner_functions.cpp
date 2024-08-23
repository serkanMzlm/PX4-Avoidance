#include "local_planner/planner_functions.hpp"

void generateNewHistogram(Histogram &polar_histogram, const Eigen::Vector3f &position,
                          const pcl::PointCloud<pcl::PointXYZ> &cropped_cloud)
{
    Eigen::MatrixXi counter(GRID_LENGTH_E, GRID_LENGTH_Z);
    counter.fill(0);
    for (auto xyz : cropped_cloud)
    {
        Eigen::Vector3f p = toEigen(xyz);
        PolarPoint p_pol = cartesianToPolarHistogram(p, position);
        float dist = p_pol.r;
        Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES);

        counter(p_ind.y(), p_ind.x()) += 1;
        polar_histogram.setDist(p_ind.y(), p_ind.x(), polar_histogram.getDist(p_ind.y(), p_ind.x()) + dist);
    }

    // Normalize and get mean in distance bins
    for (int e = 0; e < GRID_LENGTH_E; e++)
    {
        for (int z = 0; z < GRID_LENGTH_Z; z++)
        {
            if (counter(e, z) > 0)
            {
                polar_histogram.setDist(e, z, polar_histogram.getDist(e, z) / counter(e, z));
            }
            else
            {
                polar_histogram.setDist(e, z, 0.f);
            }
        }
    }
}