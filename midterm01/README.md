```bash
/src/localization/
├── CMakeLists.txt                       # 將子模組編譯為獨立庫 localization_core，供 localization 鏈接
├── config/
│   └── localization_params.yaml         # 【新增】所有演算法與系統超參數統一配置檔
├── include/localization/
│   ├── motion_predictor.h               # 【獨立庫標頭】運動學推算、初值動態濾波與姿態合理性懲罰
│   ├── coarse_to_fine_gicp.h            # 【獨立庫標頭】多尺度體素降採樣與兩階段 G-ICP 配準核心
│   └── localization.h                   # 【最上層標頭】主節點宣告，引用上述函式庫
├── src/
│   ├── motion_predictor.cpp             # 【獨立庫實作】
│   ├── coarse_to_fine_gicp.cpp          # 【獨立庫實作】
│   └── localization.cpp                 # 【最上層節點】主流程調度、子地圖裁切、檔案 I/O 與發布
└── launch/
    └── localization.launch.py           # 讀取 YAML 參數並啟動節點
```