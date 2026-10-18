# 按键状态机流程图

## 状态转移总图

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> DOWN_DEBOUNCE : PRESS\n(key_input==0)

    DOWN_DEBOUNCE --> DOWN : TIMEOUT\n(>50ms)\n回调: do_key_down
    DOWN_DEBOUNCE --> IDLE : RELEASE\n(key_input==1)\n(抖动, 忽略)

    DOWN --> UP_DEBOUNCE : RELEASE\n(key_input==1)
    DOWN --> LONGPRESSED : LONGPRESSED\n(>2000ms)\n回调: do_key_longpress

    LONGPRESSED --> LOONGPRESS_DEBOUNCE : RELEASE\n(key_input==1)

    LOONGPRESS_DEBOUNCE --> IDLE : TIMEOUT\n(>50ms)\n回调: do_key_idle
    LOONGPRESS_DEBOUNCE --> LONGPRESSED : PRESS\n(key_input==0)\n(抖动, 忽略)

    UP_DEBOUNCE --> UP : TIMEOUT\n(>50ms)\n回调: do_key_up
    UP_DEBOUNCE --> DOWN : PRESS\n(key_input==0)\n(抖动, 忽略)

    UP --> IDLE : IDLE\n(>200ms)\n回调: do_key_click
    UP --> DOUBLE_DOWN_DEBOUNCE : PRESS\n(key_input==0)

    DOUBLE_DOWN_DEBOUNCE --> DOUBLE_DOWN : TIMEOUT\n(>50ms)
    DOUBLE_DOWN_DEBOUNCE --> UP : RELEASE\n(key_input==1)\n(抖动, 忽略)

    DOUBLE_DOWN --> DOUBLE_UP_DEBOUNCE : RELEASE\n(key_input==1)
    DOUBLE_DOWN --> LONGPRESSED : LONGPRESSED\n(>2000ms)\n回调: do_key_longpress

    DOUBLE_UP_DEBOUNCE --> IDLE : TIMEOUT\n(>50ms)\n回调: do_key_double_click
    DOUBLE_UP_DEBOUNCE --> DOUBLE_DOWN : PRESS\n(key_input==0)\n(抖动, 忽略)
```

---

## 按键行为分支流程图

```mermaid
flowchart TD
    START([上电启动]) --> IDLE[空闲 KEY_STATE_IDLE]

    IDLE -->|"按下(key_input==0)"| D_DB[按下消抖 KEY_DOWN_DEBOUNCE<br>记录按下时刻]

    D_DB -->|"释放(抖动)"| IDLE
    D_DB -->|"50ms超时<br>确认按下"| DOWN[按下 KEY_STATE_DOWN<br>回调: do_key_down]

    DOWN -->|"释放"| U_DB[抬起消抖 KEY_UP_DEBOUNCE]
    DOWN -->|"2000ms超时<br>进入长按"| LP[长按 KEY_STATE_LONGPRESSED<br>回调: do_key_longpress]

    LP -->|"释放"| LP_DB[长按消抖 KEY_LOONGPRESS_DEBOUNCE]
    LP_DB -->|"按下(抖动)"| LP
    LP_DB -->|"50ms超时<br>确认释放"| IDLE_RET[返回空闲<br>回调: do_key_idle]

    U_DB -->|"按下(抖动)"| DOWN
    U_DB -->|"50ms超时<br>确认抬起"| UP[抬起 KEY_STATE_UP<br>回调: do_key_up]

    UP -->|"200ms空闲超时<br>确认为单击"| CLICK[返回空闲<br>回调: do_key_click]
    UP -->|"再次按下<br>可能为双击"| DDB[双击消抖 KEY_DOUBLE_DOWN_DEBOUNCE]

    DDB -->|"释放(抖动)"| UP
    DDB -->|"50ms超时<br>确认按下"| DD[双击按下 KEY_DOUBLE_DOWN]

    DD -->|"释放"| DUB[双击抬起消抖 KEY_DOUBLE_UP_DEBOUNCE]
    DD -->|"2000ms超时<br>转入长按"| LP

    DUB -->|"按下(抖动)"| DD
    DUB -->|"50ms超时<br>确认双击释放"| DCLICK[返回空闲<br>回调: do_key_double_click]
```

---

## 状态说明

| 状态 | 枚举值 | 含义 |
|------|--------|------|
| `KEY_STATE_IDLE` | — | 空闲状态，等待按键按下 |
| `KEY_DOWN_DEBOUNCE` | — | 按下消抖，首次检测到按下后等待 50ms 确认 |
| `KEY_STATE_DOWN` | — | 已确认按下，等待释放或长按超时 |
| `KEY_STATE_LONGPRESSED` | — | 长按状态，持续按住 2000ms 后进入 |
| `KEY_LOONGPRESS_DEBOUNCE` | — | 长按释放消抖，等待 50ms 确认释放 |
| `KEY_UP_DEBOUNCE` | — | 抬起消抖，检测到释放后等待 50ms 确认 |
| `KEY_STATE_UP` | — | 已确认抬起，等待空闲超时或再次按下 |
| `KEY_DOUBLE_DOWN_DEBOUNCE` | — | 双击第二次按下消抖，等待 50ms 确认 |
| `KEY_DOUBLE_DOWN` | — | 双击第二次已确认按下，等待释放 |
| `KEY_DOUBLE_UP_DEBOUNCE` | — | 双击抬起消抖，等待 50ms 确认释放 |

---

## 事件说明

| 事件 | 触发条件 |
|------|----------|
| `KEY_EVENT_PRESS` | `key_input() == 0`（低电平有效，按下） |
| `KEY_EVENT_RELEASE` | `key_input() == 1`（高电平，释放） |
| `KEY_EVENT_TIMEOUT` | 当前状态持续时间超过对应阈值（50 ms） |
| `KEY_EVENT_LONGPRESSED` | DOWN / DOUBLE_DOWN 状态持续超过 `KEY_LONGPRESS_MS`（2000 ms） |
| `KEY_EVENT_IDLE` | UP 状态持续超过 `KEY_IDLE_TIMEOUT_MS`（200 ms） |

---

## 时间参数

| 宏定义 | 数值 | 用途 |
|--------|------|------|
| `KEY_DEBOUNCE_MS` | 50 ms | 消抖时间，用于按下/释放确认 |
| `KEY_IDLE_TIMEOUT_MS` | 200 ms | UP 状态空闲超时，超时后确认为单击 |
| `KEY_LONGPRESS_MS` | 2000 ms | 长按判定时间，按住超过此值触发长按 |

---

## 回调函数

| 回调 | 触发时机 | 状态转移 |
|------|----------|----------|
| `do_key_down` | 按下消抖确认后 | `DOWN_DEBOUNCE → DOWN` |
| `do_key_up` | 抬起消抖确认后 | `UP_DEBOUNCE → UP` |
| `do_key_click` | UP 状态 200ms 无操作 | `UP → IDLE`（单击确认） |
| `do_key_longpress` | 长按 2000ms | `DOWN → LONGPRESSED` / `DOUBLE_DOWN → LONGPRESSED` |
| `do_key_double_click` | 双击抬起消抖确认后 | `DOUBLE_UP_DEBOUNCE → IDLE`（双击确认） |
| `do_key_idle` | 长按释放消抖确认后 | `LOONGPRESS_DEBOUNCE → IDLE` |

---

## 各回调函数触发流程分析

### 1. do_key_down — 按键按下确认

**触发条件**：首次检测到按下 → 消抖 50ms 期间持续保持按下 → 确认为有效按下。

```mermaid
flowchart LR
    S0((IDLE)) -->|"key_input==0\n(检测到按下)<br>记录 last_time"| S1[DOWN_DEBOUNCE]
    S1 -->|"key_input==1\n(抖动, 回到空闲)"| S0
    S1 -->|"持续按下 >50ms\nEVENT_TIMEOUT"| S2((DOWN<br>✅ do_key_down))
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_STATE_IDLE` |
| 触发事件 | `KEY_EVENT_PRESS`（电平 = 0）→ `KEY_DOWN_DEBOUNCE` → `KEY_EVENT_TIMEOUT`（≥ 50ms） |
| 消抖要求 | 消抖期间电平必须持续为 0，否则回到 IDLE |
| 时序 | 按下下降沿起 50ms 后触发 |

```
电平:  ───────┐     ┌────────
             │     │
             └─────┘
             | 50ms|
             ↑      ↑
         记录时刻   do_key_down ✅
```

---

### 2. do_key_up — 按键释放确认

**触发条件**：有效按下后检测到释放 → 消抖 50ms 期间持续保持释放 → 确认为有效抬起。

```mermaid
flowchart LR
    S1((DOWN)) -->|"key_input==1\n(检测到释放)"| S2[UP_DEBOUNCE]
    S2 -->|"key_input==0\n(抖动, 回到按下)"| S1
    S2 -->|"持续释放 >50ms\nEVENT_TIMEOUT"| S3((UP<br>✅ do_key_up))
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_STATE_DOWN` |
| 触发事件 | `KEY_EVENT_RELEASE`（电平 = 1）→ `KEY_UP_DEBOUNCE` → `KEY_EVENT_TIMEOUT`（≥ 50ms） |
| 消抖要求 | 消抖期间电平必须持续为 1，否则回到 DOWN |
| 时序 | 释放上升沿起 50ms 后触发 |

```
电平:  ───────┐        ┌───────
             │        │
             └────────┘
                      | 50ms|
                      ↑      ↑
                  释放时刻   do_key_up ✅
```

---

### 3. do_key_click — 单击确认

**触发条件**：有效按下 → 有效释放 → 200ms 内无再次按下 → 确认为单击。

```mermaid
flowchart LR
    S1((DOWN)) -->|"key_input==1"| S2[UP_DEBOUNCE]
    S2 -->|"持续释放 >50ms\nEVENT_TIMEOUT"| S3((UP<br>✅ do_key_up))
    S3 -->|"200ms 内再次按下\n→ 双击路径"| S4[DOUBLE_DOWN_DEBOUNCE]
    S3 -->|"200ms 无操作\nEVENT_IDLE"| S5((IDLE<br>✅ do_key_click))
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_STATE_UP` |
| 前置条件 | 已完成一次完整的按下→释放流程（do_key_down → do_key_up 已触发） |
| 触发事件 | `KEY_EVENT_IDLE`（在 UP 状态停留 ≥ 200ms 无再次按下） |
| 互斥条件 | 若 200ms 内再次按下，进入双击路径，不会触发 click |

```
电平:  ───┐      ┌──────────────────
         │      │
         └──────┘
          |  50ms|      |  200ms  |
                       ↑          ↑
                   do_key_up   do_key_click ✅
```

---

### 4. do_key_longpress — 长按确认

**触发条件（路径 A）**：有效按下后持续按住 ≥ 2000ms。
**触发条件（路径 B）**：双击第二次按下后持续按住 ≥ 2000ms。

```mermaid
flowchart TD
    subgraph 路径A [路径 A: 首次按下进入长按]
        A0((IDLE)) -->|"按下"| A1[DOWN_DEBOUNCE]
        A1 -->|"50ms 消抖"| A2((DOWN<br>✅ do_key_down))
        A2 -->|"持续按住\n≥ 2000ms\nEVENT_LONGPRESSED"| A3((LONGPRESSED<br>✅ do_key_longpress))
    end

    subgraph 路径B [路径 B: 双击第二次按下后长按]
        B0((IDLE)) -->|"按下"| B1[DOWN_DEBOUNCE]
        B1 -->|"50ms"| B2((DOWN))
        B2 -->|"释放"| B3[UP_DEBOUNCE]
        B3 -->|"50ms"| B4((UP))
        B4 -->|"再次按下"| B5[DOUBLE_DOWN_DEBOUNCE]
        B5 -->|"50ms"| B6((DOUBLE_DOWN))
        B6 -->|"持续按住\n≥ 2000ms\nEVENT_LONGPRESSED"| B7((LONGPRESSED<br>✅ do_key_longpress))
    end
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_STATE_DOWN` 或 `KEY_DOUBLE_DOWN` |
| 触发事件 | `KEY_EVENT_LONGPRESSED`（在当前状态停留 ≥ 2000ms） |
| 时序 | 进入 DOWN / DOUBLE_DOWN 后开始计时，2000ms 后触发 |

```
电平:  ───┐                              ┌─────
         │                              │
         └──────────────────────────────┘
          |  50ms|     | 1950ms |       |
                  ↑              ↑
             do_key_down   do_key_longpress ✅
```

---

### 5. do_key_double_click — 双击确认

**触发条件**：有效按下 → 有效释放 → 200ms 内再次按下 → 再次释放 → 消抖完成。

```mermaid
flowchart LR
    S1((DOWN<br>do_key_down)) -->|"释放"| S2[UP_DEBOUNCE]
    S2 -->|"50ms"| S3((UP<br>do_key_up))
    S3 -->|"<200ms 内\n再次按下"| S4[DOUBLE_DOWN_DEBOUNCE]
    S4 -->|"持续按下 >50ms"| S5((DOUBLE_DOWN))
    S5 -->|"释放"| S6[DOUBLE_UP_DEBOUNCE]
    S6 -->|"持续释放 >50ms\nEVENT_TIMEOUT"| S7((IDLE<br>✅ do_key_double_click))
    S6 -->|"再次按下(抖动)"| S5
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_DOUBLE_UP_DEBOUNCE` |
| 前置条件 | 第一次按下→释放完成（已进入 UP），<200ms 内检测到第二次按下 |
| 触发事件 | `KEY_EVENT_TIMEOUT`（第二次释放消抖 ≥ 50ms） |
| 关键约束 | 两次按下间隔 < 200ms；双击第二次若持续 ≥ 2000ms 则转为长按 |

```
电平:  ───┐    ┌──┐    ┌──────────
         │    │  │    │
         └────┘  └────┘
          |50ms|  |<200| |50ms|
                              ↑
                      do_key_double_click ✅
```

---

### 6. do_key_idle — 长按释放后空闲

**触发条件**：长按状态下释放按键 → 消抖 50ms 确认释放 → 回到空闲。

```mermaid
flowchart LR
    S1((LONGPRESSED<br>do_key_longpress)) -->|"key_input==1\n(检测到释放)"| S2[LOONGPRESS_DEBOUNCE]
    S2 -->|"key_input==0\n(抖动, 回到长按)"| S1
    S2 -->|"持续释放 >50ms\nEVENT_TIMEOUT"| S3((IDLE<br>✅ do_key_idle))
```

| 条件 | 要求 |
|------|------|
| 初始状态 | `KEY_LOONGPRESS_DEBOUNCE` |
| 前置条件 | 必须先进入 LONGPRESSED 状态（即触发了 do_key_longpress）并释放按键 |
| 触发事件 | `KEY_EVENT_TIMEOUT`（释放消抖 ≥ 50ms） |
| 互斥条件 | 消抖期间若再次按下，回到 LONGPRESSED，不触发 idle |

```
电平:  ───┐                        ┌─────
         │                        │
         └────────────────────────┘
                                 |50ms|
                                 ↑     ↑
                              释放    do_key_idle ✅
```

---

### 回调触发条件速查表

| 回调 | 物理操作 | 触发时刻 | 不可同时触发 |
|------|----------|----------|-------------|
| **do_key_down** | 按下并保持 50ms | 消抖确认后立即 | — |
| **do_key_up** | 释放并保持 50ms | 消抖确认后立即 | — |
| **do_key_click** | 短按后松手 | 松手后 200ms | 双击（互斥，由 200ms 窗口区分） |
| **do_key_longpress** | 按住不松 | 按下后 2000ms | 单击/双击（一旦长按，松手后触发 do_key_idle 而非 click/dblclick） |
| **do_key_double_click** | 短按→松手→再短按→松手 | 第二次松手消抖后 | 单击（先到先得） |
| **do_key_idle** | 长按后松手 | 松手消抖后 | 仅限长按路径 |

---

## 典型按键时序

### 单击
```
电平:  ─────┐     ┌──────────
           │     │
           └─────┘
状态: IDLE → D_DB → DOWN → U_DB → UP → IDLE(click)
时间:       0ms    50ms   释放   50ms   +200ms
                                ↑ do_key_down
                                      ↑ do_key_up
                                              ↑ do_key_click
```

### 长按
```
电平:  ─────┐                        ┌─────
           │                        │
           └────────────────────────┘
状态: IDLE → D_DB → DOWN → ... → LONGPRESSED → LP_DB → IDLE(idle)
时间:       0ms   50ms   +2000ms   +2000ms      释放   +50ms
                         ↑ do_key_longpress              ↑ do_key_idle
```

### 双击
```
电平:  ───┐    ┌──┐    ┌──────────
         │    │  │    │
         └────┘  └────┘
状态: IDLE → D_DB → DOWN → U_DB → UP → DDB → DD → DUB → IDLE(dblclick)
时间:       0ms   50ms   释放   50ms   按下  50ms  释放  50ms
                                                         ↑ do_key_double_click
```
