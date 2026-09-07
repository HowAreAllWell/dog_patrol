import re
import html
from datetime import datetime

# =======================================================
# 预编译日志清洗正则表达式 (提升高频刷屏日志的解析性能)
# =======================================================
RE_LAUNCH_PREFIX = re.compile(r'^\[[\w\-]+-\d+\]\s*')
RE_NODE_SUFFIX = re.compile(r'\[([a-zA-Z0-9_]+)-\d+\]:')
RE_UNIX_TS = re.compile(r'\[(1\d{9}\.\d+)\]')
RE_HAS_TS = re.compile(r'\[\d{2}:\d{2}:\d{2}\.\d{3}\]')
RE_LEVEL_TAG = re.compile(r'(\[(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\s*\])', flags=re.IGNORECASE)

CATEGORY_COLORS = {
    "SYS": "#FFFFFF",
    "MISSION": "#4DD0E1",
    "BUILD": "#B39DDB",
    "CAMERA": "#9C27B0",
    "LIDAR": "#F48FB1",
    "MAP": "#8BC34A",
    "LOC": "#03A9F4",
    "NAV": "#FF9100",
    "PERCEPTION": "#26A69A",
    "BAG": "#FF9800",
    "RVIZ": "#9E9E9E",
    "ALL": "#E0E0E0"
}

class LogEngine:
    def __init__(self, append_callback):
        self.append_callback = append_callback
        self.last_node_names = {}

    def parse_and_append_log(self, line, prefix="", is_stderr=False):
        # 决定所属分类 (Category) 先提前提取，方便上下文追踪
        raw_prefix = prefix.strip()
        category = "ALL"
        if "[SYS]" in raw_prefix: category = "SYS"
        elif "[MISSION]" in raw_prefix: category = "MISSION"
        elif "[BUILD]" in raw_prefix: category = "BUILD"
        elif "[CAMERA]" in raw_prefix: category = "CAMERA"
        elif "[LIDAR]" in raw_prefix: category = "LIDAR"
        elif "[MAP]" in raw_prefix: category = "MAP"
        elif "[LOC]" in raw_prefix: category = "LOC"
        elif "[NAV]" in raw_prefix: category = "NAV"
        elif "[PERCEPTION]" in raw_prefix: category = "PERCEPTION"
        elif "[BAG]" in raw_prefix: category = "BAG"
        elif "[RVIZ]" in raw_prefix: category = "RVIZ"

        # 屏蔽无法在源码层面修改的系统级动态库警告 (如 PCL 点云格式不匹配警告)
        if "Failed to find match for field" in line:
            return
            
        # 剥离 ros2 launch 产生的无意义进程框头 (如行首的 [lidar_loop_node-1] )
        line = RE_LAUNCH_PREFIX.sub('', line)
        
        # 剥离 ros2 launch 伪造节点名里的序号后缀，并统一转为圆括号格式 (如把 [fastlivo_mapping-1]: 变成 (fastlivo_mapping) )
        line = RE_NODE_SUFFIX.sub(r'(\1) ', line)
        
        # 将原生日志中残存的第四前缀 [node_name]: 也统一转为圆括号 (支持点号等特殊字符)
        line = re.sub(r'\[([^\]]+)\]:\s*', r'(\1) ', line)
        
        # 针对底层第三方库强行输出的裸日志进行格式修补
        if line.strip().startswith("scale: "):
            val = line.strip().split("scale: ")[-1]
            line = f"[laserMapping]: Found parameter: scale, value: {val}"
        
        # 将所有的长串Unix时间戳(如 [1781708455.740944941])转换为人类可读的时间
        def convert_ts(match):
            ts = float(match.group(1))
            dt = datetime.fromtimestamp(ts)
            return f"[{dt.strftime('%H:%M:%S')}.{int((ts % 1) * 1000):03d}]"
            
        line = RE_UNIX_TS.sub(convert_ts, line)
        


        # --- 统一格式注入引擎 (治愈所有不规范日志) ---
        # 检查是否已包含格式化后的时间戳
        has_ts = bool(RE_HAS_TS.search(line))
        
        if not has_ts:
            now = datetime.now()
            ts_str = f"[{now.strftime('%H:%M:%S')}.{int(now.microsecond/1000):03d}]"
            
            # 尝试寻找已有的等级标签 (如 ros2 launch 打印的 [INFO] [launch]: )
            lvl_match = RE_LEVEL_TAG.search(line)
            if lvl_match:
                # 已存在等级标签，直接将时间戳插在其后
                line = line[:lvl_match.end()] + f" {ts_str}" + line[lvl_match.end():]
            else:
                upper_line_check = line.upper()
                if is_stderr:
                    level = "[ERROR]"
                    if "BUILD" in prefix and "error:" not in line.lower() and "failed" not in line.lower():
                        level = "[WARN]"
                elif "SEGMENTATION FAULT" in upper_line_check or "CORE DUMPED" in upper_line_check or "ABORTED" in upper_line_check:
                    level = "[FATAL]"
                else:
                    level = "[INFO]"
                
                # 既然废弃了第四前缀，我们不再推断节点名，直接拼上等级和时间
                last_node = self.last_node_names.get(category, "")
                if last_node:
                    line = f"{level} {ts_str} ({last_node}) {line.lstrip()}"
                else:
                    line = f"{level} {ts_str} {line.lstrip()}"
                
        # 此时 line 必然以 [LEVEL] [TIMESTAMP] 开头。如果紧接着还有个 [XXX] 框（且没有被前面的冒号规则干掉），说明是残留的第三框（节点名）
        # 或者是已经转成圆括号的节点名。
        # 我们用正则提取出圆括号里的节点名，更新到最后已知节点名
        line = re.sub(r'^(\[(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\s*\]\s*\[\d{2}:\d{2}:\d{2}\.\d{3}\])\s*\[([^\]]+)\]\s*', r'\1 (\2) ', line)
        
        node_match = re.search(r'^\[(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\s*\]\s*\[\d{2}:\d{2}:\d{2}\.\d{3}\]\s*\(([^)]+)\)', line)
        if node_match:
            self.last_node_names[category] = node_match.group(1)
            
        # 如果一句话仅仅只有前缀和节点名（即多行日志的第一行），直接跳过渲染，避免出现空白行
        content_only = re.sub(r'^\[(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\s*\]\s*\[\d{2}:\d{2}:\d{2}\.\d{3}\]\s*(?:\([^)]+\))?\s*', '', line)
        if not content_only.strip():
            return
            
        upper_line = line.upper()
        safe_line = html.escape(line)
        
        # HTML 渲染器会吃掉换行符和连续空格，所以我们需要转换为 <br> 和 &nbsp;
        safe_line = safe_line.replace('\n', '<br>')
        safe_line = safe_line.replace('  ', '&nbsp;&nbsp;')
        
        # 提取或分配前缀颜色
        prefix_color = CATEGORY_COLORS.get(category, "#E0E0E0")
        
        # 兼容旧版的 HTML <font> 提取（如果还有残留的话）
        m = re.search(r"<font color='([^']+)'>([^<]+)</font>(.*)", prefix)
        if m:
            prefix_color = m.group(1)
            raw_prefix = (m.group(2) + m.group(3)).strip()

        # 决定整条日志的最终统一颜色
        if "[ERROR]" in upper_line or "[FATAL]" in upper_line:
            final_color = "#F44336" 
        elif "[WARN]" in upper_line or "[WARNING]" in upper_line:
            final_color = "#FFC107" 
        elif "[DEBUG]" in upper_line:
            final_color = "#9E9E9E" 
        elif is_stderr and "[INFO]" not in upper_line:
            final_color = "#FF9800" 
        else:
            final_color = prefix_color # INFO或正常输出：直接继承该模块(前缀)的专属颜色
            
        # 组装带颜色的前缀字符串
        formatted_prefix = f"<font color='{prefix_color}'>{raw_prefix} </font>" if raw_prefix else ""
            
        self.append_callback(f"<font color='{final_color}'>{formatted_prefix}{safe_line}</font>", category=category)
