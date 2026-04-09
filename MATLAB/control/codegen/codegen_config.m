function [cfg_lib, cfg_mex] = codegen_config()
% CODEGEN_CONFIG Embedded Coder configuration for controller code generation.
%   cfg_lib: coder.EmbeddedCodeConfig — C static library target
%   cfg_mex: coder.MexCodeConfig      — MEX target for simulation
%   出力ディレクトリは codegen() の '-d' オプションで指定する

    %% C library config (for Rust FFI)
    cfg_lib = coder.config('lib');
    cfg_lib.TargetLang = 'C';
    cfg_lib.GenerateReport = true;
    cfg_lib.GenCodeOnly = true;  % build.rs がコンパイルを担当
    cfg_lib.SupportNonFinite = false;
    cfg_lib.EnableDynamicMemoryAllocation = false;
    cfg_lib.FilePartitionMethod = 'SingleFile';  % コントローラごとに1ファイル
    cfg_lib.GenerateExampleMain = 'DoNotGenerate';

    % ハードウェアターゲット: ARM Cortex-M (STM32F303K8)
    hw = coder.HardwareImplementation;
    hw.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    hw.TargetHWDeviceType = 'ARM Compatible->ARM Cortex-M';
    cfg_lib.HardwareImplementation = hw;

    %% MEX config (for MATLAB simulation)
    cfg_mex = coder.config('mex');
    cfg_mex.GenerateReport = true;
    cfg_mex.EnableAutoExtrinsicCalls = false;
end
