/*
<<<<<<< HEAD
 * @Description:
=======
 * @Description: 
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

<<<<<<< HEAD
#include "Geocentric/LocalCartesian.hpp"

namespace lidar_localization
{
  class GNSSData
  {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
=======
//#include <GeographicLib/LocalCartesian.hpp>
#include "Geocentric/LocalCartesian.hpp"

namespace lidar_localization {
class GNSSData {
  public:
    double time = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

<<<<<<< HEAD
  public:
    void InitOriginPosition();
    void UpdateXYZ();
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
  };
}
#endif
=======
  public: 
    void InitOriginPosition();
    void UpdateXYZ();

    static void Reverse(
      const double &local_E, const double &local_N, const double &local_U,
      double &lat, double &lon, double &alt
    );

    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}
#endif
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
