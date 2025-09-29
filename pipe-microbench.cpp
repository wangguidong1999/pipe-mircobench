// pipebench.cpp
// 单可执行文件，跑 3 组测试并输出结果。
// 依赖：common.hpp（需提供 Options、parse_options、allocate_buf、write_size_str、perf_*、log/fail）
//
// 编译：g++ -O3 -std=gnu++17 pipebench.cpp -o pipebench
// 运行：./pipebench
// 帮助：./pipebench --help
//
// 注：实现了 --runs/-n、--writer-cpu、--reader-cpu、--no-pin；绑核使用 sched_setaffinity。

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>

#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>

#include "common.hpp"

// ---------- 小工具 ----------
static struct timespec g_tspec;
static inline double now_millis() {
  if (clock_gettime(CLOCK_REALTIME, &g_tspec) < 0) {
    fail("could not get time: %s", strerror(errno));
  }
  return ((double)g_tspec.tv_sec)*1000.0 + ((double)g_tspec.tv_nsec)/1e6;
}

static inline double to_gib_per_sec(size_t bytes, double ms) {
  double gbps = (double)bytes / 1'000'000.0 / ms;   // GB/s (10^9 基)
  return gbps * 0.931323;                           // 转 GiB/s（与 pv 一致）
}

static bool pin_to_cpu(int cpu_id) {
  if (cpu_id < 0) return false;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu_id, &set);
  return sched_setaffinity(0, sizeof(set), &set) == 0;
}

static std::string basename_of(const char* argv0) {
  const char* p = strrchr(argv0, '/');
  return p ? std::string(p+1) : std::string(argv0 ? argv0 : "pipebench");
}

// --------- perf 权限检查 ---------
static bool perf_permission_ok() {
  // root 直接放行
  if (geteuid() == 0) return true;
  FILE* f = fopen("/proc/sys/kernel/perf_event_paranoid", "re");
  if (!f) {
    // 某些老系统可能没有该文件，交给后续逻辑
    return true;
  }
  long v = 0;
  int ok = fscanf(f, "%ld", &v);
  fclose(f);
  if (ok != 1) return true;
  // 常见规则：非特权用户需要 <= 1 才能打开需要的 perf 事件
  return v <= 1;
}

// ---------- 写端（来自 write.cpp） ----------
NOINLINE UNUSED
static void with_write(const Options& options, char* buf) {
  if (options.busy_loop) {
    if (fcntl(STDOUT_FILENO, F_SETFL, O_NONBLOCK) < 0)
      fail("could not mark stdout pipe as non blocking: %s", strerror(errno));
  }
  struct pollfd pfd{ .fd = STDOUT_FILENO, .events = POLLOUT | POLLWRBAND };
  while (true) {
    char* cursor = buf;
    ssize_t remaining = options.buf_size;
    while (remaining > 0) {
      if (options.poll && options.busy_loop) {
        while (poll(&pfd, 1, 0) == 0) {}
      } else if (options.poll) {
        poll(&pfd, 1, -1);
      }
      ssize_t ret = write(STDOUT_FILENO, cursor, remaining);
      if (ret < 0 && errno == EPIPE) goto finished;
      if (ret < 0 && errno == EAGAIN) continue;
      if (ret < 0) fail("write failed: %s", strerror(errno));
      cursor += ret;
      remaining -= ret;
    }
  }
finished:
  return;
}

NOINLINE UNUSED
static void with_vmsplice(const Options& options, char* bufs[2]) {
  struct pollfd pfd{ .fd = STDOUT_FILENO, .events = POLLOUT | POLLWRBAND };
  size_t buf_ix = 0;
  while (true) {
    struct iovec bufvec{ .iov_base = bufs[buf_ix], .iov_len = options.buf_size };
    buf_ix = (buf_ix + 1) % 2;
    while (bufvec.iov_len > 0) {
      if (options.poll && options.busy_loop) {
        while (poll(&pfd, 1, 0) == 0) {}
      } else if (options.poll) {
        poll(&pfd, 1, -1);
      }
      ssize_t ret = vmsplice(
        STDOUT_FILENO, &bufvec, 1,
        (options.busy_loop ? SPLICE_F_NONBLOCK : 0) | (options.gift ? SPLICE_F_GIFT : 0)
      );
      if (ret < 0 && errno == EPIPE) goto finished;
      if (ret < 0 && errno == EAGAIN) continue;
      if (ret < 0) fail("vmsplice failed: %s", strerror(errno));
      bufvec.iov_base = (void*)(((char*)bufvec.iov_base) + ret);
      bufvec.iov_len -= ret;
    }
  }
finished:
  return;
}

static void writer_run(Options options) {
  signal(SIGPIPE, SIG_IGN);
  // perf_init();

  if (options.pipe_size && options.write_with_vmsplice) {
    fail("cannot write with vmsplice and set the pipe size manually. it will be automatically determined.");
  }
  if (options.write_with_vmsplice) {
    if (options.buf_size % 2 != 0) {
      fail("if writing with vmsplice, the buffer size must be divisible by two");
    }
    options.pipe_size = options.buf_size / 2;
  }
  if (options.pipe_size > 0) {
    int fcntl_res = fcntl(STDOUT_FILENO, F_SETPIPE_SZ, options.pipe_size);
    if (fcntl_res < 0) {
      if (errno == EPERM) {
        fail("setting the pipe size failed with EPERM, %zu is probably above the pipe size limit\n", options.buf_size);
      } else {
        fail("setting the pipe size failed, are you piping the output somewhere? error: %s\n", strerror(errno));
      }
    }
    if ((size_t)fcntl_res != options.pipe_size) {
      fail("could not set the pipe size to %zu, got %d instead\n", options.pipe_size, fcntl_res);
    }
  }

  // reset_perf_count();
  // enable_perf_count();

  if (options.write_with_vmsplice) {
    char* bufs[2];
    if (options.same_buffer) {
      char* buf = allocate_buf(options);
      options.buf_size = options.buf_size / 2;
      bufs[0] = buf;
      bufs[1] = buf + options.buf_size;
    } else {
      bufs[0] = allocate_buf(options);
      bufs[1] = allocate_buf(options);
    }
    with_vmsplice(options, bufs);
  } else {
    char* buf = allocate_buf(options);
    with_write(options, buf);
  }

  // disable_perf_count();
  // log_perf_count();
  // perf_close();
}

// ---------- 读端（修正 pollfd.fd=STDIN_FILENO） ----------
NOINLINE
static size_t with_read(const Options& options, char* buf) {
  if (options.busy_loop) {
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) < 0)
      fail("could not mark stdin pipe as non blocking: %s", strerror(errno));
  }
  struct pollfd pfd;
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN | POLLPRI;

  size_t read_count = 0;
  while (read_count < options.bytes_to_pipe) {
    if (options.poll && options.busy_loop) {
      while (poll(&pfd, 1, 0) == 0) {}
    } else if (options.poll) {
      poll(&pfd, 1, -1);
    }
    ssize_t ret = read(STDIN_FILENO, buf, options.buf_size);
    if (ret < 0 && errno == EAGAIN) continue;
    if (ret < 0) fail("read failed: %s", strerror(errno));
    if (ret == 0) break; // pipe closed
    read_count += ret;
  }
  return read_count;
}

NOINLINE
static size_t with_splice(const Options& options) {
  struct pollfd pfd;
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN | POLLPRI;

  size_t read_count = 0;
  int devnull = open("/dev/null", O_WRONLY);
  if (devnull < 0) fail("open /dev/null failed: %s", strerror(errno));

  while (read_count < options.bytes_to_pipe) {
    if (options.poll && options.busy_loop) {
      while (poll(&pfd, 1, 0) == 0) {}
    } else if (options.poll) {
      poll(&pfd, 1, -1);
    }
    ssize_t ret = splice(
      STDIN_FILENO, NULL, devnull, NULL, options.buf_size,
      (options.busy_loop ? SPLICE_F_NONBLOCK : 0) | (options.gift ? SPLICE_F_MOVE : 0)
    );
    if (ret < 0 && errno == EAGAIN) continue;
    if (ret < 0) fail("splice failed: %s", strerror(errno));
    if (ret == 0) break;
    read_count += ret;
  }
  close(devnull);
  return read_count;
}

static size_t reader_run(const Options& options) {
  if (options.read_with_splice) return with_splice(options);
  char* buf = allocate_buf(options);
  return with_read(options, buf);
}

// ---------- 单次测试（带 fork/pipe 与可选绑核） ----------
struct OneRun {
  double gibps = 0.0;
};

struct RunnerCfg {
  int writer_cpu = 0;
  int reader_cpu = 1;
  bool pin = false;
};

static OneRun run_once(Options wopt, Options ropt, const RunnerCfg& rcfg) {
  int fds[2];
  if (pipe(fds) < 0) fail("pipe failed: %s", strerror(errno));

  pid_t pid = fork();
  if (pid < 0) fail("fork failed: %s", strerror(errno));

  if (pid == 0) {
    // 子：写端
    close(fds[0]);
    if (fds[1] != STDOUT_FILENO) {
      if (dup2(fds[1], STDOUT_FILENO) < 0) fail("dup2 stdout failed: %s", strerror(errno));
      close(fds[1]);
    }
    if (rcfg.pin) (void)pin_to_cpu(rcfg.writer_cpu);
    writer_run(wopt);
    _exit(0);
  }

  // 父：读端
  close(fds[1]);
  if (fds[0] != STDIN_FILENO) {
    if (dup2(fds[0], STDIN_FILENO) < 0) fail("dup2 stdin failed: %s", strerror(errno));
    close(fds[0]);
  }
  if (rcfg.pin) (void)pin_to_cpu(rcfg.reader_cpu);

  double t0 = now_millis();
  size_t got = reader_run(ropt);
  double t1 = now_millis();

  close(STDIN_FILENO);
  int status = 0;
  (void)waitpid(pid, &status, 0);

  OneRun r;
  r.gibps = to_gib_per_sec(got, t1 - t0);
  return r;
}

// ---------- 5 组定义 ----------
struct CaseDef {
  const char* label; // 用于单次行输出的短标签
  bool w_vmsplice;
  bool r_splice;
  bool huge;
  bool busy;
};
static const CaseDef kCases[5] = {
  {"baseline",                         false, false, false, false},
  {"vmsplice",                         true,  false, false, false},
  {"vmsplice+splice",                  true,  true,  false, false},
  {"vmsplice+splice+hugepage",         true,  true,  true,  false},
  {"vmsplice+splice+hugepage+busy",    true,  true,  true,  true },
};

static void apply_common_sanity(Options& w, Options& r) {
  r.buf_size       = w.buf_size       = (size_t)std::max(w.buf_size,       r.buf_size);
  r.bytes_to_pipe  = w.bytes_to_pipe  = (size_t)std::max(w.bytes_to_pipe,  r.bytes_to_pipe);
  if (w.write_with_vmsplice && (w.buf_size % 2)) { // vmsplice 需要偶数（双缓冲对半）
    ++w.buf_size; r.buf_size = w.buf_size;
  }
}

// ---------- CLI 解析（仅解析新增控制项；其他仍交给 common.hpp 的 parse_options） ----------
struct Cli {
  int runs = 3;
  int writer_cpu = 0;
  int reader_cpu = 1;
  bool pin = false;
  bool help = false;
};

static Cli parse_cli_only(int argc, char** argv) {
  Cli c;
  bool cpu_specified = false;
  bool no_pin = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need_val = [&](const char* opt)->const char*{
      if (i+1 >= argc) { fprintf(stderr, "option %s requires an argument\n", opt); exit(1); }
      return argv[++i];
    };
    if (a == "-n" || a == "--runs") {
      c.runs = std::max(1, atoi(need_val(a.c_str())));
    } else if (a == "--writer-cpu") {
      c.writer_cpu = atoi(need_val("--writer-cpu"));
      cpu_specified = true;
    } else if (a == "--reader-cpu") {
      c.reader_cpu = atoi(need_val("--reader-cpu"));
      cpu_specified = true;
    } else if (a == "--no-pin") {
      // c.pin = false;
      no_pin = true;
    } else if (a == "--help" || a == "-h") {
      c.help = true;
    }
  }
  c.pin = cpu_specified && !no_pin;
  return c;
}

static void print_help(const char* argv0) {
  std::string bn = basename_of(argv0);
  // printf("提示：需用sudo执行；或执行命令允许非特权用户打开涉及内核的perf事件（sudo sysctl -w kernel.perf_event_paranoid=1）\n");
  printf("用法: ./%s [选项]\n", bn.c_str());
  printf("-n, --runs N            每组重复次数（默认 3）\n");
  printf("--writer-cpu ID         写端绑核（默认不绑核）\n");
  printf("--reader-cpu ID         读端绑核（默认不绑核）\n");
  printf("--no-pin                强制禁用绑核（不使用 sched_setaffinity）\n");
  printf("示例:\n");
  printf("./%s -n 5 --writer-cpu 2 --reader-cpu 3\n", bn.c_str());
}

// ---------- 主流程 ----------
int main(int argc, char** argv) {
  // 先解析自定义的 CLI（不会让 common.hpp 看到这些参数）
  Cli cli = parse_cli_only(argc, argv);
  if (cli.help) {        // 先拦截 --help，避免传给 parse_options 报错
    print_help(argv[0]);
    return 0;
  }

  // perf 权限检查：避免后面跑出一堆 "Error opening leader" 与 0.000 结果
  // if (!perf_permission_ok()) {
  //   fprintf(stderr,
  //     "提示：需用sudo执行；或允许非特权用户打开会涉及内核的perf事件（sudo sysctl -w kernel.perf_event_paranoid=1）\n");
  //   return 2; // 非零退出，表示权限不足
  // }
  
  // 从 argv 中剥离自定义的参数，只把剩余参数交给 parse_options
  std::vector<char*> filtered;
  filtered.reserve(argc);
  filtered.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto skip_val = [&]() {
      if (i + 1 < argc) ++i;
    };
    if (a == "-n" || a == "--runs" ||
        a == "--writer-cpu" || a == "--reader-cpu") {
      skip_val();                 // 跳过它的参数值
      continue;                   // 不传给 parse_options
    }
    if (a == "--no-pin" || a == "--help" || a == "-h") {
      continue;                   // 这些是无参开关，也不传
    }
    filtered.push_back(argv[i]);  // 其余保留，交给 common.hpp
  }
  int argc2 = (int)filtered.size();
  char** argv2 = filtered.data();

  // 解析基础 Options（bytes、buf 等仍沿用 common.hpp）
  Options base;
  parse_options(argc2, argv2, base);

  RunnerCfg rcfg{ cli.writer_cpu, cli.reader_cpu, cli.pin };

  // 标题块
  std::cout << "[Pipes speed test]\n";
  std::cout << "pipes microbench  | runs=" << cli.runs << "\n";
  // std::cout << "pinning: writer_cpu=" << rcfg.writer_cpu
  //           << "  reader_cpu=" << rcfg.reader_cpu << (rcfg.pin ? "" : "  (disabled)") << "\n\n";
  if (rcfg.pin) {
    std::cout << "pinning: writer_cpu=" << rcfg.writer_cpu
              << "  reader_cpu=" << rcfg.reader_cpu << "\n\n";
  } else {
    std::cout << "pinning: (disabled)\n\n";
  }
  // 逐组逐次运行并打印
  std::vector<double> medians(5, 0.0);
  for (int ci = 0; ci < 5; ++ci) {
    const CaseDef& C = kCases[ci];
    Options w = base, r = base;
    w.write_with_vmsplice = C.w_vmsplice ? 1 : 0;
    r.read_with_splice    = C.r_splice   ? 1 : 0;
    w.huge_page = r.huge_page = C.huge ? 1 : 0;
    w.busy_loop = r.busy_loop = C.busy ? 1 : 0;
    apply_common_sanity(w, r);

    std::vector<double> runs;
    runs.reserve(cli.runs);
    for (int k = 0; k < cli.runs; ++k) {
      OneRun r1 = run_once(w, r, rcfg);
      runs.push_back(r1.gibps);

      // 行输出（宽度对齐到 29）
      std::ostringstream tag;
      tag << "[" << std::left << std::setw(29) << C.label << "]";
      std::cout << tag.str()
                << " run " << (k+1) << ": "
                << std::fixed << std::setprecision(3) << r1.gibps << " GiB/s\n";
    }
    // 计算 median
    std::sort(runs.begin(), runs.end());
    double median = runs.size() % 2 ? runs[runs.size()/2]
                                    : 0.5*(runs[runs.size()/2 - 1] + runs[runs.size()/2]);
    medians[ci] = median;
  }

  // 汇总表
  std::cout << "\nCase                                 | Median GiB/s | x vs base \n";
  std::cout << "-------------------------------------|--------------|----------\n";
  double base_med = medians[0] > 0 ? medians[0] : 1.0;
  for (int ci = 0; ci < 5; ++ci) {
    const CaseDef& C = kCases[ci];
    double med = medians[ci];
    double mult = med / base_med;
    std::cout << std::left << std::setw(37) << C.label
              << "| " << std::right << std::setw(12) << std::fixed << std::setprecision(3) << med
              << " | x" << std::left << std::setw(9) << std::fixed << std::setprecision(2) << mult
              << "\n";
  }

  // std::cout << "\n单独执行命令如下：\n";
  // std::cout << "./write | ./read\n";
  // std::cout << "./write --write_with_vmsplice | ./read\n";
  // std::cout << "./write --write_with_vmsplice | ./read --read_with_splice\n";
  // std::cout << "./write --write_with_vmsplice --huge_page | ./read --read_with_splice\n";
  // std::cout << "./write --write_with_vmsplice --huge_page --busy_loop | ./read --read_with_splice --busy_loop\n";
  std::cout << "\n测试说明：\n";
  std::cout << "- baseline：普通write()/read()通过管道传输，存在用户态/内核态之间两次拷贝。（体现默认管道性能）\n";
  std::cout << "- vmsplice：写端用vmsplice将用户页“赠与”管道，消除写端用户->内核拷贝；读端仍read()，保留一次拷贝。\n";
  std::cout << "- vmsplice+splice：读端改用splice，写端vmsplice；双零拷贝，用户态参与最少。（接近管道实现上限）\n";
  std::cout << "- vmsplice+splice+hugepage：双零拷贝基础上启用大页，降低TLB/页表开销。\n";
  std::cout << "- vmsplice+splice+hugepage+busy：配合非阻塞+忙轮询，减少调度/唤醒延迟。（逼近系统上限）\n";
  // std::cout << "注：倍率为相对baseline的提升倍数；具体数值受内核版本、CPU/内存拓扑、频率策略等影响。\n";
  return 0;
}

