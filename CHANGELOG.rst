^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_bumper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* updated readme, updated ci
* fix launch, env->optenv
* Remove pcl warnings
* Contributors: Tomas Baca, vojta

1.0.3 (2022-05-09)
------------------
* added custom config option
* refactored against publisher handler
* refactored agains updated transformer
* + install in cmakelists
* removed flag override
* Contributors: Tomas Baca

1.0.2 (2021-10-03)
------------------
* add missing ros shutdown request
* Contributors: Pavel Petracek, Tomas Baca

1.0.1 (2021-05-16)
------------------
* disabled debugging flags
* updated ros::shutdown
* changed the bumper default vertical fov
* added config parameters for using include_box and exclude_box to simulation.yaml
* added include_box filter to lidar3d data, fixed voxelgrid
* Contributors: Matouš Vrba, Tomas Baca, Vaclav Pritzl

1.0.0 (2021-03-18)
------------------
* Major release

0.0.2 (2021-03-16)
------------------
* changed 3d lidar topic to os_cloud_nodelet/points_processed
* cleanup, minor fixes, added console output
* modification for 3D lidar
* thread-timer -> ros::Timer
* fixed the mrs_lib API, refactored launch and config files
* increased fallback num sectors
* Contributors: Matej Petrlik, Matej Petrlik (desktop), Matouš Vrba, Robert Penicka, Tomas Baca, Tomáš Báča, Vaclav Pritzl, Viktor Walter, Vojtech Spurny, afzal, klaxalk, mergify[bot], uav43, uav45, uav71

0.0.1 (2019-05-20)
------------------
