def grid_path_to_lonlat(grid_path, proj_enu, scale):
    lonlat_path=[]
    for xg, yg, zg in grid_path:
        x = xg/scale
        y = yg/scale
        lon, lat = proj_enu(x, y, inverse=True)
        lonlat_path.append((lon, lat, zg))
    return lonlat_path
