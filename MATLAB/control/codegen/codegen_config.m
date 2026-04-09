function [cfg_lib, cfg_mex] = codegen_config(lib_output_dir, mex_output_dir)
% CODEGEN_CONFIG Embedded Coder configuration for controller code generation.
%   cfg_lib: coder.EmbeddedCodeConfig — C static library target
%   cfg_mex: coder.MexCodeConfig      — MEX target for simulation

    %% C library config (for Rust FFI)
    cfg_lib = coder.config('lib');
    cfg_lib.TargetLang = 'C';
    cfg_lib.GenerateReport = true;
    cfg_lib.GenCodeOnly = true;  % build.rs がコンパイルを担当
    cfg_lib.SupportNonFinite = false;
    cfg_lib.DynamicMemoryAllocation = 'Off';
    cfg_lib.FilePartitionMethod = 'SingleFile';  % コントローラごとに1ファイル
    cfg_lib.GenerateExampleMain = false;

    % ハードウェアターゲット: ARM Cortex-M (STM32F303K8)
    hw = coder.HardwareImplementation;
    hw.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    hw.TargetHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    cfg_lib.HardwareImplementation = hw;

    % 出力ディレクトリ
    cfg_lib.CodeGenDirectory = lib_output_dir;

    %% MEX config (for MATLAB simulation)
    cfg_mex = coder.config('mex');
    cfg_mex.GenerateReport = true;
    cfg_mex.EnableAutoExtrinsicCalls = false;

    % MEX出力ディレクトリ
    cfg_mex.CodeGenDirectory = mex_output_dir;
end
