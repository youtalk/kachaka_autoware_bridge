# Kachaka × Autoware Core 統合設計

- 作成日: 2026-05-02
- ステータス: ドラフト（実装計画前のレビュー段階）
- 関連: [autoware_core](https://github.com/autowarefoundation/autoware_core), [kachaka-api](https://github.com/pf-robotics/kachaka-api), [autoware_rviz_plugins](https://github.com/autowarefoundation/autoware_rviz_plugins)

## 1. 目的

Kachaka を Autoware Core のパッケージで自律移動させる。Kachakaが既に持っている内蔵 navigation を経由せず、Autoware の localization・planning・control がそのまま動くロボットとして扱えるようにすることで、Autoware エコシステムの資産（lanelet2 / NDT / mission_planner / pure_pursuit / AD-API / autoware_rviz_plugins）を Kachaka に持ち込む。

## 2. スコープ

### 2.1 In Scope（MVP）

「**RViz の Autoware 標準UI で 2D Goal Pose を 1 点指定すると、Kachaka が Autoware の planning と control を経由して目標位置に到達する**」までを MVP とする。具体的には:

- Localization: Autoware NDT + EKF を Ouster OS-1 128 で動作
- Planning: lanelet2 vector_map を前提に `autoware_core_planning` をフル動作
- Control: `autoware_simple_pure_pursuit` の出力を Kachakaの差動駆動 Twist に変換
- AD-API: `autoware_default_adapi` + `autoware_adapi_adaptors` + `autoware_rviz_plugins` を採用
- Vehicle Interface: 独立した新規パッケージとして実装し、将来的な完全 Autoware vehicle_interface 準拠への発展余地を残す

### 2.2 Out of Scope（MVP外、後続フェーズ）

- 動的物体検出（perception スタック）
- 巡回 / 複数 waypoint
- 障害物停止（`motion_velocity_planner::ObstacleStopModule` の有効化は MVP では off）
- Kachaka の dock/undock シーケンスとの連携
- Kachaka の内蔵 navigation の完全置換（kachaka_command 系）
- マルチKachaka（`frame_prefix` の使用）

## 3. 前提と制約

### 3.1 物理構成

- **「Autoware シェルフ」**を 1 つ用意し、その上に **Jetson Thor** と **Ouster OS-1 128** を固定する。
- このシェルフは Kachaka に **常時搭載されている運用前提**（MVPではドッキング遷移を扱わない）。
- Kachaka 本体は IP 192.168.1.91 で gRPC API（port 26400）を提供。
- Thor 上で Autoware 一式と `kachaka_grpc_ros2_bridge` を動かす。
- 開発 PC 上で RViz2 + autoware_rviz_plugins を動かし、Thor と同じ ROS_DOMAIN_ID で通信。
- **Kachakaの 2D LiDAR は故障しているため使用しない**。Kachaka 内蔵の地図作成・自己位置推定は当てにせず、Autoware NDT が唯一の自己位置推定手段。Kachaka の `/scan` トピックも購読しない。

### 3.2 ソフトウェア前提

- ROS 2 Jazzy（Thor / 開発PC）
- ROS_DOMAIN_ID = 123
- Autoware Core ワークスペース: `~/ros/jazzy`
- Kachaka SW バージョン: `kachaka-api` 3.16 以降
- GNSS は使えない（屋内）
- 屋内のため `pose_initializer` の `gnss_enabled: false`、`yabloc_enabled: false`

### 3.3 事前作業

MVP に入る前に以下が完了している必要がある（M0 マイルストーン）:

1. **pointcloud_map の作成**: **OS-1 128 単独で**自宅をマッピング（lio_sam, fast_lio, glim 等）し、`pointcloud_map.pcd` + `pointcloud_map/metadata.yaml` を生成する。Kachaka の 2D LiDAR は故障しているため補助に使えない。マッピング時は Kachaka を手押し or テレオペで動かすか、別途 OS-1 を手で持って歩く。
2. **lanelet2 vector_map の作成**: TIER IV の Vector Map Builder で自宅の通行可能領域に最小限のレーンを引き、`lanelet2_map.osm` + `map_projector_info.yaml` を生成する。**pointcloud_map と lanelet2 vector_map は同一の local projection 原点で生成**する（座標系の整合は §4.3 を参照）。
3. **Ouster OS-1 のシェルフへの物理固定** とキャリブレーション値（base_footprint → os1_sensor の static transform）の取得。

### 3.4 設計判断（合意済み）

| 項目 | 選択 | 備考 |
|---|---|---|
| スコープ | フル Autoware（C） | localization 含む |
| MVP | 1 点指定移動（B） | RViz 1 操作で Kachaka 到達 |
| マップ | lanelet2 手動作成（A） | Vector Map Builder |
| Vehicle Interface | 独立ノード（B、将来 C 拡張） | Operation Mode 状態機械含む |
| 初期姿勢 | NDT monte carlo（D） | + autoware_rviz_plugins |
| AD-API | フル導入（A） | default_adapi + adaptors + rviz_plugins |

## 4. システム構成

### 4.1 ノード配置

```
┌────────────── Kachaka 本体 (192.168.1.91) ──────────────┐
│  gRPC API :26400                                         │
│   - SetRobotVelocity (Twist 入力)                         │
│   - SetManualControlEnabled (Autoware Engage 時 true)      │
│   - GetRosOdometry / GetRosImu / GetRosLaserScan ...       │
└──────────────────────────────────────────────────────────┘
        ↑ gRPC over WiFi/Ethernet
        │
┌────── Autoware シェルフ（Kachaka に常時搭載） ──────────┐
│  Jetson Thor (Ubuntu 24.04 + ROS 2 Jazzy)                 │
│   ├─ Ouster OS-1 128 (シェルフ天面)                        │
│   ├─ ouster-ros driver                                    │
│   ├─ kachaka_grpc_ros2_bridge (既存)                       │
│   ├─ autoware_core_map / localization / planning / control │
│   ├─ autoware_default_adapi + autoware_adapi_adaptors      │
│   └─ kachaka_autoware_bridge / vehicle_interface (新規)     │
└──────────────────────────────────────────────────────────┘
        │ DDS (ROS_DOMAIN_ID=123)
        │
┌────── 開発PC（Jazzy）─────────────────────────────────────┐
│  RViz2 + autoware_rviz_plugins                             │
│  (InitialPoseButtonPanel / RouteTool / EngageButton 等)     │
└──────────────────────────────────────────────────────────┘
```

### 4.2 TF ツリー

```
map  (NDT scan matcher)
 └─ odom  (autoware_ekf_localizer が publish)
     └─ base_footprint
         └─ base_link  (Kachakaの kachaka_description)
             ├─ base_r_drive_wheel_link
             ├─ base_l_drive_wheel_link
             ├─ ... (既存)
             └─ docking_link  (既存 prismatic、ドッキング・リフト機構)
                 └─ shelf_base_link  (新規 fixed、純正 3 段シェルフの底面中心)
                     ├─ shelf_bottom_board / middle_board / top_board
                     ├─ shelf_fl_post / fr_post / bl_post / br_post
                     └─ shelf_top  (シェルフ上面、ペイロード取付アンカー)
                         └─ os1_sensor  (新規 fixed, Ouster OS-1 128)
                             ├─ os1_imu
                             └─ os1_lidar
```

`map → odom`: `autoware_ekf_localizer` が public（既存）
`odom → base_footprint`: Kachakaの `dynamic_tf_bridge`（既存）
`base_footprint → base_link → ... → docking_link`: Kachakaの `_kachaka.urdf.xacro` + `robot_state_publisher`（既存、無変更）。`docking_link` フレームは `base_link` 原点に置かれ、prismatic joint で 0–0.012 m リフトする。
`docking_link → shelf_base_link`: `kachaka_description/urdf/_shelf_3tier.urdf.xacro` の `shelf_3tier` マクロを呼ぶときに **`<origin xyz="0 0 0.115"/>` を渡してソレノイド上面（cylinder center 0.1075 + half-length 0.0075）にシェルフ底面を載せる**。これでドッキング・リフト時もシェルフが追従する。マクロ自体は `*origin` を受け取れるよう改良済み。
`shelf_base_link → shelf_*`: 同 `_shelf_3tier.urdf.xacro`（**改良で追加**、純正 3 段シェルフはKachakaの装備品なので kachaka_description の責務）
`shelf_top → os1_sensor → os1_lidar/imu`: `kachaka_autoware_description/urdf/_ouster_os1.urdf.xacro`（新規パッケージ）

### 4.3 座標系の整合

- pointcloud_map と lanelet2 vector_map は **同一の local projection 原点** で生成する（Vector Map Builder の `map_projector_info.yaml` の origin と、SLAM ツールで pointcloud_map を作る際の世界原点を一致させる）。
- Kachaka の内部 map 座標とは独立に Autoware の map 座標を運用する（一致は要求しない）。Kachaka の `/odometry`（odom 相対）の相対変位はそのまま EKF の `vehicle_velocity_converter` 経由で利用するが、Kachaka の絶対 pose は使わない。
- これにより、pointcloud_map とlanelet2 が共通原点で揃っていれば NDT が出す map 座標がそのまま planning と整合する。

## 5. 新規パッケージ構成

すべて kachaka-api リポジトリの `ros2/` 配下に置き、Kachaka本家のリリースサイクルに乗せる。

```
ros2/
├── kachaka_description/                            # ★ 改良: 純正 3 段シェルフのマクロを追加
│   └── urdf/
│       └── _shelf_3tier.urdf.xacro                  # 新規: 3 段シェルフのマクロ。Kachaka装備品なのでこのパッケージに置く
│   (既存ファイル _materials / _values / _kachaka 等は破壊変更しない)
│
├── kachaka_autoware_bridge/                        # メタ + 統合launch
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── launch/
│       └── kachaka_autoware.launch.xml              # 全部入りエントリポイント
├── kachaka_autoware_vehicle_interface/              # ★ 中核（C++ Vehicle Interface ノード）
│   ├── src/
│   │   ├── vehicle_interface_node.cpp               # Control→Twist + status + operation_mode
│   │   ├── vehicle_interface_node.hpp
│   │   ├── operation_mode_state_machine.cpp         # 簡易 mode decider
│   │   └── operation_mode_state_machine.hpp
│   ├── launch/vehicle_interface.launch.xml
│   ├── config/vehicle_interface.param.yaml
│   └── test/
│       └── test_control_to_twist.cpp
├── kachaka_autoware_description/                    # OS-1 + シェルフ装着済の完全 URDF + vehicle_info.yaml
│   ├── urdf/
│   │   ├── _ouster_os1.urdf.xacro                   # OS-1 128 のマクロ（簡易円筒モデル + lidar/imu フレーム）
│   │   └── kachaka_with_shelf.urdf.xacro            # kachaka + 3 段シェルフ + OS-1 の完全 URDF
│   ├── config/
│   │   └── vehicle_info.param.yaml                  # 差動駆動向け仮想値
│   └── launch/robot_description.launch.py
└── kachaka_autoware_maps/                           # サンプルマップ配置先（実体は外部）
    └── README.md                                     # ユーザー作成手順
```

新規パッケージの依存関係:

- `kachaka_autoware_vehicle_interface` → `autoware_control_msgs`, `autoware_vehicle_msgs`, `autoware_adapi_v1_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`
- `kachaka_autoware_description` → `kachaka_description`（改良後）, `xacro`, `robot_state_publisher`
- `kachaka_autoware_bridge` → 上記 3 つ + `autoware_default_adapi`, `autoware_adapi_adaptors`, `autoware_core_*`

`kachaka_description` の改良:
- `urdf/_shelf_3tier.urdf.xacro` を追加: `xacro:macro name="shelf_3tier" params="parent shelf_name"` を提供
- `urdf/_materials.urdf.xacro` にシェルフ用マテリアル（`shelf_board` / `shelf_post`）追加
- 既存の `_kachaka.urdf.xacro` / `_values.urdf.xacro` / `kachaka.urdf.xacro` には触らない（破壊変更を避け、既存ユーザーの URDF 出力を変えない）

## 6. Localization 設計

### 6.1 入力

- OS-1 128: `ouster-ros` driver で `/sensing/lidar/top/pointcloud_raw_ex` (`sensor_msgs/PointCloud2`) として発行
- Kachaka wheel odometry: 既存 `wheel_odometry_component` の `/kachaka/wheel_odometry/wheel_odometry`
  - Vehicle Interface が `/vehicle/status/velocity_status` (`autoware_vehicle_msgs/VelocityReport`) として再発行
  - `vehicle_velocity_converter` が `/sensing/vehicle_velocity_converter/twist_with_covariance` に変換
  - **要検証**: Kachaka 内部で `wheel_odometry` がホイールエンコーダ＋IMU ベースで生成されている前提だが、もし内部実装が 2D LiDAR にも依存していると故障の影響を受ける可能性がある。M2 立ち上げ時に実機で確認し、もし使えない場合は IMU (`/kachaka/imu/imu`) と Kachaka の `wheel_odometry` の角速度成分のみから自前で twist を組み立てる代替経路を実装する

### 6.2 構成

`autoware_core_localization.launch.xml` をほぼそのまま使い、以下のパラメータ調整のみ行う:

- `pose_initializer.param.yaml`:
  - `gnss_enabled: false`
  - `yabloc_enabled: false`
  - `ndt_enabled: true`
  - `pose_error_check_enabled: false`
  - `stop_check_enabled: true`
- `ndt_scan_matcher.param.yaml`:
  - `initial_pose_estimation.particles_num` を実機チューニング
  - `align_using_monte_carlo: true`（全範囲探索を許可）
- GNSS topic はノードを起動しない（`gnss_enabled: false` で `pose_initializer` 側は GNSS をブロックしないため、`/sensing/gnss/pose_with_covariance` への空 publisher は不要）。`autoware_core_localization.launch.xml` への launch arg `gnss_input_topic` は空文字に上書き

### 6.3 出力

- `/localization/kinematic_state` (`nav_msgs/Odometry`) — 下流で唯一参照される真実
- `/localization/acceleration` (`geometry_msgs/AccelWithCovarianceStamped`)
- TF: `map → odom`

## 7. Planning 設計

`autoware_core_planning.launch.xml` をほぼそのまま使い、以下を調整:

- `vehicle_param_file`: 新規 `kachaka_autoware_description/config/vehicle_info.param.yaml`
- `motion_velocity_planner_launch_modules`: MVPでは `[]`（ObstacleStop 無効）。M6 で有効化する際は **OS-1 128 由来の点群** を `/perception/obstacle_segmentation/pointcloud` に流す（Kachaka の 2D LiDAR は故障しているため使えない）。地面除去は `autoware_crop_box_filter` 等で対応。
- 入力 perception トピック（`/perception/object_recognition/objects`, `/perception/obstacle_segmentation/pointcloud`, `/perception/traffic_light_recognition/traffic_signals`, `/perception/occupancy_grid_map/map` 等）は M4 立ち上げ時に **実機検証** する。`behavior_velocity_planner` / `motion_velocity_planner` が起動できないトピックがあれば `kachaka_autoware_bridge` 内で空メッセージを 1 Hz で publish する補助ノード `perception_stub` を追加する（M4 で必要性判定）。

### 7.1 ゴール受信フロー

1. RViz `RouteTool` (autoware_rviz_plugins) が `/api/routing/set_route_points` を呼ぶ
2. `autoware_default_adapi/routing` が変換して `/planning/mission_planning/set_waypoint_route` を呼ぶ
3. `mission_planner` が lanelet2 上で route を計算して `/planning/route` を発行
4. `path_generator` → `behavior_velocity_planner` → `motion_velocity_planner` → `velocity_smoother` → `/planning/trajectory`

## 8. Control 設計

`autoware_simple_pure_pursuit` を **そのまま使用**。fork しない。

- 入力: `/localization/kinematic_state`, `/planning/trajectory`
- 出力: `/control/command/control_cmd` (`autoware_control_msgs/Control`)
- `vehicle_info` の `wheel_base` は差動駆動に対する仮想値（後述）

## 9. Vehicle Interface 設計（中核）

### 9.1 ノード概要

`kachaka_vehicle_interface_node` は単一のノード（`rclcpp::Node`）として実装し、以下の責務を持つ。

#### A. Control → Twist 変換

- 購読: `/control/command/control_cmd`
- 自転車モデル → 差動駆動への変換式:
  - `v = control.longitudinal.velocity`
  - `omega = v * tan(control.lateral.steering_tire_angle) / wheel_base`
- 出力: `geometry_msgs/Twist`
- リミット: `|v| ≤ 0.3 m/s`, `|omega| ≤ 1.57 rad/s`（既存 ManualControl の上限と整合）

#### B. cmd_vel publish ゲート

- `/system/operation_mode/state` を購読し、`mode == AUTONOMOUS` のときだけ `/kachaka/manual_control/cmd_vel` に Twist を流す。
- それ以外のモードでは publish 自体を停止。

#### C. /vehicle/status/velocity_status publish

- 購読: Kachakaの `wheel_odometry`（既存）または `/odometry/odometry`
- 変換: `nav_msgs/Odometry` → `autoware_vehicle_msgs/VelocityReport`
  - `longitudinal_velocity = twist.linear.x`
  - `lateral_velocity = 0.0`（差動駆動）
  - `heading_rate = twist.angular.z`
- 周期: 50 Hz

#### D. Manual Control 自動有効化

- ノード起動時に Kachakaの `/kachaka/manual_control/set_enabled` サービスを `true` で呼ぶ
- ノード停止時には `false` で呼ぶ（destructor / on_shutdown）

#### E. Operation Mode 状態機械

`autoware_default_adapi` の `autoware_core` 版には `operation_mode` ノードが含まれないため、Vehicle Interface 内に簡易状態機械を実装する:

- 状態: `STOP` / `AUTONOMOUS`（MVPではこの2つだけ）
- 提供サービス: `/system/operation_mode/change_to_autonomous`, `/system/operation_mode/change_to_stop`
- 発行トピック: `/system/operation_mode/state` (`autoware_adapi_v1_msgs/OperationModeState`) を 10 Hz で publish
- 起動時の初期状態は `STOP`、RViz EngageButton 経由で `AUTONOMOUS` に遷移
- 将来 C スコープに発展させる際は、このノードを `autoware_command_mode_decider`（Universe）に置き換え可能なよう、責務を 9.1 のサブモジュール構造で分離して実装する

### 9.2 設定

`config/vehicle_interface.param.yaml`:

```yaml
/**:
  ros__parameters:
    max_linear_velocity: 0.3
    max_angular_velocity: 1.57
    cmd_vel_timeout: 0.5         # [sec] これを超えたら停止
    publish_period_velocity_status: 0.02  # 50 Hz
    publish_period_operation_mode: 0.1     # 10 Hz
    auto_enable_manual_control: true
```

### 9.3 vehicle_info.param.yaml（差動駆動向け仮想値）

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.045
    wheel_width: 0.025
    wheel_base: 0.30          # 仮想値、pure_pursuit の旋回係数として機能
    wheel_tread: 0.20         # Kachaka URDF 実値
    front_overhang: 0.237      # body collision +X: 0.0435 + 0.387/2
    rear_overhang: 0.150       # body collision -X: -(0.0435 - 0.387/2)
    left_overhang: 0.120       # body collision +Y: 0 + 0.240/2
    right_overhang: 0.120      # body collision -Y: -(0 - 0.240/2)
    vehicle_height: 1.20      # シェルフ込み
    max_steer_angle: 1.5708
```

`wheel_base` は実機チューニング項目。0.30 を初期値とし、旋回が鋭すぎる場合は大きく、鈍い場合は小さくする。

## 10. AD-API 連携

### 10.1 起動

`autoware_core_api.launch.xml` をそのまま使う:

- `autoware_default_adapi` (`interface` / `localization` / `routing`)
- `autoware_adapi_adaptors` (`initial_pose_adaptor` / `routing_adaptor`)

### 10.2 RViz パネル

`autoware_rviz_plugins` を別途 clone・build:

```bash
cd ~/ros/jazzy/src
git clone https://github.com/autowarefoundation/autoware_rviz_plugins.git
cd ~/ros/jazzy
colcon build --packages-select autoware_rviz_plugins
```

採用パネル:

| パネル | 役割 |
|---|---|
| `InitialPoseButtonPanel` | 初期姿勢の確定（NDT monte carlo 起動） |
| `RouteTool`（または標準 `2D Goal Pose`） | ゴール指定 → `/api/routing/set_route_points` |
| `EngageButton` | AUTONOMOUS 遷移 |
| `AutowareStatePanel` | operation_mode 表示 |

### 10.3 操作シーケンス（MVP）

1. Thor 上で `kachaka_autoware.launch.xml` を起動
2. 開発 PC で RViz2 起動、`autoware.rviz` 設定読込
3. `InitialPoseButtonPanel` で「Initialize」ボタン押下 → NDT monte carlo で初期姿勢確定
4. `RouteTool` で目標位置指定 → trajectory 生成確認
5. `EngageButton` で AUTONOMOUS 遷移 → Vehicle Interface が cmd_vel を流し始め Kachaka が移動
6. ゴール到達 → trajectory が短くなり pure_pursuit が停止指令、自動的に STOP モードに戻る

## 11. データフロー（要点図）

```
OS-1 → /sensing/lidar/top/pointcloud_raw_ex
        ↓ (downsample) NDT scan_matcher
        ↓
        EKF ← Kachaka wheel_odometry → vehicle_velocity_converter
        ↓
   /localization/kinematic_state
        ↓
        Planning (mission_planner ← AD-API set_route_points)
        ↓
   /planning/trajectory
        ↓
   simple_pure_pursuit
        ↓
   /control/command/control_cmd  (autoware_control_msgs/Control)
        ↓
   kachaka_vehicle_interface
        ↓ (operation_mode == AUTONOMOUS のみ)
   /kachaka/manual_control/cmd_vel  (geometry_msgs/Twist)
        ↓ gRPC SetRobotVelocity
   Kachaka 本体
```

## 12. エラー処理 / フェイルセーフ

| 異常 | 検出 | 動作 |
|---|---|---|
| Control msg 受信タイムアウト | Vehicle Interface が `cmd_vel_timeout` 超過 | ゼロ Twist を 1 秒間 publish して停止 |
| Operation Mode != AUTONOMOUS | Vehicle Interface | publish 停止（cmd_vel 流さない） |
| Kachaka SetRobotVelocity 拒否 | gRPC `kErrorCodeApiGrpcSetRobotVelocityNotInTeleopMode` | 既存 `ManualControlComponent` のリトライ |
| NDT スコア悪化 | （MVP外）`exe_time_ms` / score 監視 | M6 で実装、当面は人手監視 |
| ノード単独 crash | rclcpp 標準 | systemd / launch lifecycle で再起動（運用設計） |

## 13. テスト戦略

### 13.1 単体テスト（gtest）

- `kachaka_autoware_vehicle_interface`:
  - Control → Twist 変換の境界値（v=0, δ=0, δ=max）
  - Operation Mode 状態遷移（STOP → AUTONOMOUS → STOP）
  - cmd_vel timeout 時のゼロ Twist 発行
  - velocity_status 変換

### 13.2 結合テスト

- ros2 bag を録っておき（OS-1 + Kachaka odometry + control_cmd）、オフライン再生で NDT/EKF 出力の回帰テスト
- AD-API シナリオテスト（`autoware_default_adapi/test` を参考）

### 13.3 システムテスト（実機）

- 自宅環境で 2D Goal Pose を 5 箇所、各 3 回試行
- 成功条件: Goal Pose ± 0.3 m / ± 0.2 rad 以内に到達、人手介入なし
- 失敗時のログ: rosbag フル録画

## 14. 段階的ビルドアップ（マイルストーン）

| ID | 内容 | 完了条件 |
|---|---|---|
| **M0** | 事前作業 | OS-1 単独で pointcloud_map.pcd 作成 / lanelet2_map.osm 作成（同一原点）/ OS-1 物理固定 + キャリブ |
| **M1** | センサー統合 | OS-1 が ROS 2 で発行、TF ツリー完成、RViz で base_footprint 基準の点群が見える |
| **M2** | Localization | NDT + EKF が `/localization/kinematic_state` を出す（Kachaka 静止状態で確認）。`wheel_odometry` の妥当性を実機確認、NG なら IMU フォールバック実装 |
| **M3** | Vehicle Interface 基盤 | Control→Twist 変換、velocity_status、operation_mode 状態機械、ManualControl 自動有効化 |
| **M4** | Planning | mission_planner→trajectory 生成（手動 trigger） |
| **M5** | 閉ループ | AD-API + RViz から 1 点指定で Kachaka が移動（**MVP 達成**） |
| **M6** | 仕上げ | OS-1 由来の障害物停止 / 複数waypoint / dock 連携（後続フェーズ） |

## 15. 将来拡張への余地（B → C へ）

Vehicle Interface のサブモジュール分離（Control 変換 / velocity_status / operation_mode）により、将来 Universe の `autoware_command_mode_decider` 等に置き換える際は **Operation Mode 状態機械サブモジュールだけ** 差し替えれば良い。Control 変換と velocity_status は Kachaka 固有なので残る。

## 16. リスクと未確定事項

| 項目 | リスク | 対応 |
|---|---|---|
| `wheel_base` 仮想値のチューニング | 旋回特性が直感に合わない可能性 | M5 で実機値調整、param.yaml に明記 |
| pointcloud_map 作成の手間 | M0 で時間がかかる、Kachakaの 2D LiDAR が壊れているため OS-1 単独で行うしかない | lio_sam / fast_lio / glim 経験者の知見をリサーチ。手押し / テレオペでマッピング走行 |
| 屋内 NDT のロバスト性 | 特徴の少ない壁面で divergence。Kachakaの SLAM を fallback に使えない | M2 で実機評価、必要なら voxel_size 調整。NDT 単独失敗時のフェイルセーフは AUTONOMOUS 自動解除（pose error チェック）で対応 |
| Kachaka `wheel_odometry` の妥当性 | 2D LiDAR 故障の影響で内部融合がおかしい可能性 | M2 で実機検証。NG なら IMU + 角速度から twist を自前生成する代替経路を実装 |
| OS-1 128 + Thor の発熱・電源 | シェルフ運用での連続動作 | 別途熱・電源設計（本仕様書の範囲外） |
| Kachaka 内蔵 navigation との競合 | gRPC 側で独自に動き出す可能性 | `set_manual_control_enabled(true)` で抑止 |
| 障害物停止の代替センサーが OS-1 のみ | 2D LiDAR 故障により Kachakaの近接センサ群を活用できない | M6 で OS-1 128 の点群を地面除去 → ObstacleStop 入力。地面除去パラメータが屋内向きにチューニングが必要 |

## 17. オープンな質問（実装前に決めたいが本仕様書では未確定）

- Vehicle Interface ノードを Thor 上のどのプロセスに置くか（`grpc_ros2_bridge_container` に同居 vs 独立 container）
- Operation Mode 簡易状態機械の `change_to_*` サービスを **複数同時呼び出し** された場合のセマンティクス
- pointcloud_map のサイズが大きい場合の Thor メモリ運用
