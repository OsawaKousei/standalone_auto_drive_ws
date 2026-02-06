# ステータスレポート

## 実施内容

- 共有エラー/結果型を [src/shared/result.hpp](src/shared/result.hpp) に、データモデルを [src/shared/types.hpp](src/shared/types.hpp) に追加。
- エントリポイントを [src/main.cpp](src/main.cpp) で共有型と検証ロジックを使う形に更新。
- [src/features](src/features) 配下に各機能のインターフェースと最小実装の雛形を作成：
  - Planning: [planning/i_planner.hpp](src/features/planning/i_planner.hpp), [planning/astar_planner.hpp](src/features/planning/astar_planner.hpp), [planning/astar_planner.cpp](src/features/planning/astar_planner.cpp)
  - Control: [control/i_controller.hpp](src/features/control/i_controller.hpp), [control/pure_pursuit.hpp](src/features/control/pure_pursuit.hpp), [control/pure_pursuit.cpp](src/features/control/pure_pursuit.cpp)
  - Localization: [localization/i_localizer.hpp](src/features/localization/i_localizer.hpp), [localization/ransac_localizer.hpp](src/features/localization/ransac_localizer.hpp), [localization/ransac_localizer.cpp](src/features/localization/ransac_localizer.cpp)
  - Simulation: [simulation/i_physics.hpp](src/features/simulation/i_physics.hpp), [simulation/unicycle_model.hpp](src/features/simulation/unicycle_model.hpp), [simulation/unicycle_model.cpp](src/features/simulation/unicycle_model.cpp), [simulation/i_sensor.hpp](src/features/simulation/i_sensor.hpp), [simulation/lidar_sim.hpp](src/features/simulation/lidar_sim.hpp), [simulation/lidar_sim.cpp](src/features/simulation/lidar_sim.cpp)
  - Visualization: [visualization/visualizer.hpp](src/features/visualization/visualizer.hpp), [visualization/visualizer.cpp](src/features/visualization/visualizer.cpp)
- [CMakeLists.txt](CMakeLists.txt#L1-L33) を更新し新規ソースを追加、プロジェクト名を変更。
- [README.md](README.md) を自律走行計画に合わせて更新。

## 現状

- ロードマップ Step 1（基盤）の雛形が揃った段階。ビルド実施済。
