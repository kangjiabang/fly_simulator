from matplotlib import pyplot as plt


def plot_3d(path_grid, buildings, start_ll, goal_ll, proj_enu, scale, nofly_zones):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 1. 绘制路径
    if path_grid:
        xs = [p[0] / scale for p in path_grid]
        ys = [p[1] / scale for p in path_grid]
        zs = [p[2] for p in path_grid]
        ax.plot(xs, ys, zs, color='blue', linewidth=3, label='Planned Path', zorder=10)

    # 2. 绘制建筑 (处理 MultiPolygon)
    for b in buildings:
        geom = b["geometry"]
        # 获取高度，兼容不同键名
        z_min = b.get("min_alt", 0)
        z_max = b.get("max_alt", b.get("height", 50))

        # 将 MultiPolygon 统一视为几何列表处理
        parts = geom.geoms if hasattr(geom, 'geoms') else [geom]

        for part in parts:
            # 提取外轮廓坐标
            lons, lats = part.exterior.xy
            # 投影到 ENU 坐标系
            enu_coords = [proj_enu(lon, lat) for lon, lat in zip(lons, lats)]
            px, py = zip(*enu_coords)

            # 绘制顶部和底部多边形轮廓
            ax.plot(px, py, [z_max] * len(px), color='brown', alpha=0.7)
            ax.plot(px, py, [z_min] * len(px), color='brown', alpha=0.3)

            # 绘制垂直边柱 (侧墙)
            for i in range(len(px)):
                ax.plot([px[i], px[i]], [py[i], py[i]], [z_min, z_max],
                        color='brown', alpha=0.3, linestyle='--')

        # 4. 绘制禁飞区
    for z in nofly_zones:
        geom = z["geometry"]
        z_min = z["z_min"]
        z_max = z["z_max"]

        lons, lats = geom.exterior.xy
        enu = [proj_enu(lon, lat) for lon, lat in zip(lons, lats)]
        px, py = zip(*enu)

        ax.plot(px, py, [z_max] * len(px),
                color='red', linewidth=2, linestyle='--', alpha=0.8)
        ax.plot(px, py, [z_min] * len(px),
                color='red', linewidth=1, alpha=0.4)

        for i in range(len(px)):
            ax.plot([px[i], px[i]], [py[i], py[i]],
                    [z_min, z_max],
                    color='red', alpha=0.3)
    # 3. 绘制起终点
    start_enu = proj_enu(start_ll[0], start_ll[1])
    goal_enu = proj_enu(goal_ll[0], goal_ll[1])
    ax.scatter([start_enu[0]], [start_enu[1]], [start_ll[2]], color='green', s=100, label='Start')
    ax.scatter([goal_enu[0]], [goal_enu[1]], [goal_ll[2]], color='red', s=100, label='Goal')

    # 设置标签和视角
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude (m)")

    # 调整视角：elev=30度仰角，azim=-60度方位角，这样看3D效果更好
    # 如果想从正上方看，请改回 (90, -90)
    #ax.view_init(elev=30, azim=-60)
    ax.view_init(elev=90, azim=-90)
    plt.legend()
    plt.title("3D Path Planning with Buildings")
    plt.show()
