from line_profiler import LineProfiler

def profile_each_line(func):
    """自定义装饰器：手动触发行级性能分析"""
    def wrapper(*args, **kwargs):
        lp = LineProfiler()
        lp.add_function(func)
        # 如果该方法内部调用了 get_nofly_zones，也可以一并加入分析
        # from .some_module import get_nofly_zones
        #lp.add_function(draw_pic) 
        
        lp.enable_by_count()
        try:
            return lp(func)(*args, **kwargs)
        finally:
            lp.disable_by_count()
            # 在控制台打印结果
            lp.print_stats()
            # (可选) 导出到文件
            # with open("profile_results.txt", "a") as f:
            #     lp.print_stats(stream=f)
    return wrapper
