import warnings
from deprecated import deprecated

def custom_formatwarning(message, category, filename, lineno, file=None, line=None):
    return f"\033[33mWarning: {filename}:{lineno} - {message}\033[0m\n"

warnings.formatwarning = custom_formatwarning

def sdk_deprecated(reason=None, version=None, replacement=None, removed_in=None, remove_date=None):
    """
    SDK专用的弃用装饰器，支持函数和类
    
    Args:
        reason (str): 弃用原因
        version (str): 弃用版本
        replacement (str): 替代函数/类名，如果为None则自动生成
        removed_in (str): 将在哪个版本移除
        remove_date (str): 将在哪个日期移除 (格式: YYYY-MM-DD)
    """
    # 构建reason字符串，包含replacement、removed_in和remove_date信息
    if reason is None:
        reason = ""
    if replacement:
        if reason:
            reason += f"，建议使用 {replacement} 替代"
        else:
            reason = f"建议使用 {replacement} 替代"
    if removed_in:
        if reason:
            reason += f"，将在版本 {removed_in} 移除"
        else:
            reason = f"将在版本 {removed_in} 移除"
    if remove_date:
        if reason:
            reason += f"，预计移除日期: {remove_date}"
        else:
            reason = f"预计移除日期: {remove_date}"
    
    # 使用标准的deprecated装饰器
    return deprecated(version=version, reason=reason)

