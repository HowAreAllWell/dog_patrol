# -*- coding: utf-8 -*-
# C++ 括号配平检查：字符级状态机（正确处理 字符串/字符字面量/行注释/块注释
# 及转义），避免 "file://" 里的 // 被误当注释的剥离式实现缺陷。
import sys

FILES = [
    'src/reid_node.cpp',
    'src/reid_cloud_proxy_node.cpp',
    'src/reid_gallery.cpp',
    'src/cloud_library.cpp',
    'src/feature_extractor.cpp',
    'src/feature_extractor_trt.cpp',
    'include/dog_patrol_reid_provider/reid_gallery.hpp',
    'include/dog_patrol_reid_provider/request_tracker.hpp',
    'include/dog_patrol_reid_provider/cloud_library.hpp',
    'include/dog_patrol_reid_provider/feature_extractor.hpp',
    'include/dog_patrol_reid_provider/feature_extractor_trt.hpp',
    'test/test_gallery_main.cpp',
    'test/sim_e2e_main.cpp',
]

CODE, STR, CHR, LINE_C, BLOCK_C = range(5)


def brace_delta(src):
    counts = {'{': 0, '}': 0, '(': 0, ')': 0, '[': 0, ']': 0}
    state = CODE
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ''
        if state == CODE:
            if c == '"' and nxt == '"':        # 空串字面量
                i += 2
                continue
            if c == '"':
                state = STR
            elif c == "'":
                state = CHR
            elif c == '/' and nxt == '/':
                state = LINE_C
                i += 2
                continue
            elif c == '/' and nxt == '*':
                state = BLOCK_C
                i += 2
                continue
            elif c in counts:
                counts[c] += 1
        elif state == STR:
            if c == '\\':
                i += 2
                continue
            if c == '"':
                state = CODE
        elif state == CHR:
            if c == '\\':
                i += 2
                continue
            if c == "'":
                state = CODE
        elif state == LINE_C:
            if c == '\n':
                state = CODE
        elif state == BLOCK_C:
            if c == '*' and nxt == '/':
                state = CODE
                i += 2
                continue
        i += 1
    return counts, state


ok = True
for f in FILES:
    src = open(f, encoding='utf-8').read()
    counts, end_state = brace_delta(src)
    bad = [o for o, c in [('{', '}'), ('(', ')'), ('[', ']')]
           if counts[o] != counts[c]]
    if bad or end_state != CODE:
        print(f, 'MISMATCH' if bad else 'UNTERMINATED',
              {o: (counts[o], counts[c]) for o, c in
               [('{', '}'), ('(', ')'), ('[', ']')] if counts[o] != counts[c]},
              'end_state=%d' % end_state)
        ok = False
    else:
        print(f, 'OK')
sys.exit(0 if ok else 1)
