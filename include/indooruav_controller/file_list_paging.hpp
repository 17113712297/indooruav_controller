#ifndef INDOORUAV_CONTROLLER_FILE_LIST_PAGING_HPP
#define INDOORUAV_CONTROLLER_FILE_LIST_PAGING_HPP

// ─────────────────────────────────────────────────────────────────────────────
// 文件列表分页（0x64 / 0x6E 请求某一页，0x94 / 0x95 回一页）
//
// 背景：PSDK 低速通道单帧 payload 只能到 251 B（LEN 是 1 字节），
// 而 `mode_manager` 的 list_maps / list_waypoints 返回的是**整串**文件名。
// 以前 controller_hardware 直接 `min(size, 240)` 截断后一帧发完，文件一多
// 列表就被砍掉大半，遥控器上只能看到前 7~12 条。
//
// 现在改成按"条"分页：请求里带 offset（已收到的条数），回包带
//   [offset:u16][next:u16][name\nname\n...]
// next = 0xFFFF 表示没有下一页。
//
// 本文件只依赖 <string>/<vector>/<cstdint>，不含 DJI/ROS 头，
// 方便在 PC 上用 g++ 单测。
// ─────────────────────────────────────────────────────────────────────────────

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace indooruav_paging {

//! next 字段的哨兵：没有下一页了
constexpr uint16_t kNoMore = 0xFFFF;

//! 单帧 payload 预算 240 B，其中 4 B 是 [offset][next] 头
constexpr std::size_t kMaxTextBytes = 236;

struct Page {
    uint16_t    offset = 0;       //!< 本页起始条数索引
    uint16_t    next   = kNoMore; //!< 下一页的起始索引；kNoMore 表示已到末尾
    std::size_t count  = 0;       //!< 本页条目数
    std::string text;             //!< 以 '\n' 分隔的文件名，无结尾分隔符
};

inline bool IsListSeparator(char c) {
    return c == ',' || c == '\n' || c == '\r';
}

inline std::string Trim(const std::string& s) {
    std::size_t begin = 0;
    std::size_t end   = s.size();
    while (begin < end && (s[begin] == ' ' || s[begin] == '\t')) ++begin;
    while (end > begin && (s[end - 1] == ' ' || s[end - 1] == '\t')) --end;
    return s.substr(begin, end - begin);
}

/**
 * 把 mode_manager 返回的整串（逗号分隔，也兼容换行）拆成文件名列表。
 * 空串 → 空列表（目录里没有文件）。
 */
inline std::vector<std::string> SplitList(const std::string& raw) {
    std::vector<std::string> names;
    std::string current;
    for (char c : raw) {
        if (IsListSeparator(c)) {
            std::string name = Trim(current);
            if (!name.empty()) names.push_back(name);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    std::string tail = Trim(current);
    if (!tail.empty()) names.push_back(tail);
    return names;
}

/**
 * 从 names 里取第 offset 条开始的一页，保证：
 *   1. 只按整条切分，绝不把文件名切成两半；
 *   2. text 长度不超过 max_text_bytes（默认 236）；
 *   3. next 精确指向下一条，越过末尾则为 kNoMore。
 *
 * 单个文件名超过 max_text_bytes 的极端情况：独占一页并截断，
 * 调用方应同时打 warning（正常命名不会发生）。
 */
inline Page BuildPage(const std::vector<std::string>& names,
                      uint16_t offset,
                      std::size_t max_text_bytes = kMaxTextBytes) {
    Page page;
    page.offset = offset;

    const std::size_t total = names.size();
    if (offset >= total) return page;  // 空页 + kNoMore

    for (std::size_t i = offset; i < total; ++i) {
        const std::string& name = names[i];
        const std::size_t extra = (page.count == 0) ? name.size() : 1 + name.size();

        if (page.count > 0 && page.text.size() + extra > max_text_bytes) break;

        if (page.count == 0 && name.size() > max_text_bytes) {
            page.text  = name.substr(0, max_text_bytes);
            page.count = 1;
            break;
        }

        if (page.count > 0) page.text.push_back('\n');
        page.text += name;
        ++page.count;
    }

    const std::size_t next = static_cast<std::size_t>(offset) + page.count;
    page.next = (next >= total) ? kNoMore : static_cast<uint16_t>(next);
    return page;
}

/**
 * 把一页打包成完整的 0xAA 帧：
 *   [0xAA][cmd][len][offset:u16 LE][next:u16 LE][text][XOR]
 * len = 4 + text.size()，XOR 覆盖前 3+len 字节（与既有协议一致）。
 *
 * 常量用参数传入，避免本头文件依赖 controller_hardware.hpp（那会拖进 DJI/ROS 头）。
 */
inline std::vector<std::uint8_t> BuildFrame(std::uint8_t cmd,
                                            std::uint8_t frame_header,
                                            const Page& page) {
    std::vector<std::uint8_t> frame;
    const std::size_t payload_len = 4 + page.text.size();
    frame.reserve(4 + payload_len);

    frame.push_back(frame_header);
    frame.push_back(cmd);
    frame.push_back(static_cast<std::uint8_t>(payload_len));
    frame.push_back(static_cast<std::uint8_t>(page.offset & 0xFF));
    frame.push_back(static_cast<std::uint8_t>((page.offset >> 8) & 0xFF));
    frame.push_back(static_cast<std::uint8_t>(page.next & 0xFF));
    frame.push_back(static_cast<std::uint8_t>((page.next >> 8) & 0xFF));
    frame.insert(frame.end(), page.text.begin(), page.text.end());

    std::uint8_t xor_val = 0;
    for (std::size_t i = 0; i < frame.size(); ++i) xor_val ^= frame[i];
    frame.push_back(xor_val);
    return frame;
}

/**
 * 校验一帧（自测与联调时用）：返回 true 表示帧头/长度/XOR 都对。
 */
inline bool VerifyFrame(const std::vector<std::uint8_t>& frame,
                        std::uint8_t frame_header) {
    if (frame.size() < 8) return false;
    if (frame[0] != frame_header) return false;
    const std::size_t payload_len = frame[2];
    if (frame.size() != 4 + payload_len) return false;
    std::uint8_t xor_val = 0;
    for (std::size_t i = 0; i + 1 < frame.size(); ++i) xor_val ^= frame[i];
    return xor_val == frame.back();
}

}  // namespace indooruav_paging

#endif  // INDOORUAV_CONTROLLER_FILE_LIST_PAGING_HPP
