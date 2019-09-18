clc;
clear;  
close all;

for M1 = 1:100 % 循环次数
%% 参数设置
% Params:
USE_WARPLAB_TXRX = 1;   % Enable WARPLab-in-the-loop (otherwise sim-only) 启用WARPLab-in-the-loop（否则仅限sim）
WRITE_PNG_FILES  = 0;   % Enable writing plots to PNG  启用将图形写入PNG
CHANNEL          = 11;  % Channel to tune Tx and Rx radios 选择发送机和接收机的使用的信道

N_OFDM_SYMS      = 40;  % Number of OFDM symbols OFDM技术实际发送的信号数量
MOD_ORDER        = 16;  % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM) 采用的调制方式
TX_SCALE         = 1.0; % Scale for Tx waveform ([0:1]) 缩放Tx波形
SC_IND_PILOTS  = [8 22 44 58];                        % Pilot subcarrier indices 导频子载波序号
SC_IND_DATA    = [2:7 9:21 23:27 39:43 45:57 59:64];  % Data subcarrier indices  数据子载波序号
N_SC_IND_DATA  = length(SC_IND_DATA); 
N_SC           = 64;                                  % Number of subcarriers 子载波总数
CP_LEN         = 16;                                  % Cyclic prefix length  OFDM循环前缀的长度
N_DATA_SYMS    = N_OFDM_SYMS * 48;                    % @发送OFDM数据符号数
N_DATA_SYMS_Copy = N_DATA_SYMS;                       % 备份N_DATA_SYMS的数据
INTERP_RATE    = 2;                                   % Interpolation rate (must be 2) 插值率（必须为2）
FFT_OFFSET                    = 4;           % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation LTS相关性的归一化阈值
DO_APPLY_CFO_CORRECTION       = 1;           % Enable CFO estimation/correction 启用CFO估算/更正
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction 启用剩余CFO估算/更正
DO_APPLY_SFO_CORRECTION       = 1;           % Enable SFO estimation/correction 启用SFO估计/更正
DECIMATE_RATE                 = INTERP_RATE;
USE_AGC = true;
MAX_TX_LEN = 2^20; 
TRIGGER_OFFSET_TOL_NS = 3000;
if(USE_WARPLAB_TXRX)
    NUMNODES = 2;
    nodes   = wl_initNodes(NUMNODES);
    node_tx = nodes(1);
    node_rx = nodes(2);
    eth_trig = wl_trigger_eth_udp_broadcast;
    wl_triggerManagerCmd(nodes, 'add_ethernet_trigger', [eth_trig]);
    trig_in_ids  = wl_getTriggerInputIDs(nodes(1));
    trig_out_ids = wl_getTriggerOutputIDs(nodes(1));
    wl_triggerManagerCmd(nodes, 'output_config_input_selection', [trig_out_ids.BASEBAND, trig_out_ids.AGC], [trig_in_ids.ETH_A]);
    nodes.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.BASEBAND], 0);
    nodes.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.AGC], TRIGGER_OFFSET_TOL_NS);
    ifc_ids_TX = wl_getInterfaceIDs(node_tx);
    ifc_ids_RX = wl_getInterfaceIDs(node_rx);
    TX_RF     = ifc_ids_TX.RF_A;
    TX_RF_VEC = ifc_ids_TX.RF_A;
    TX_RF_ALL = ifc_ids_TX.RF_ALL;
    
    RX_RF     = ifc_ids_RX.RF_A;
    RX_RF_VEC = ifc_ids_RX.RF_A;
    RX_RF_ALL = ifc_ids_RX.RF_ALL;

    wl_interfaceCmd(node_tx, TX_RF_ALL, 'channel', 2.4, CHANNEL);
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'channel', 2.4, CHANNEL);
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_gains', 3, 30); 
    
    if(USE_AGC)
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'automatic');
        wl_basebandCmd(nodes, 'agc_target', -13);
    else
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'manual');
        RxGainRF = 2;                  % Rx RF Gain in [1:3]
        RxGainBB = 12;                 % Rx Baseband Gain in [0:31]
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gains', RxGainRF, RxGainBB);
    end

    % Get parameters from the node 从节点获取参数 - 采样频率
    SAMP_FREQ = wl_basebandCmd(nodes(1), 'tx_buff_clk_freq'); % ? *SAMP_FREQ是指采样频率吗？
    Ts        = 1/SAMP_FREQ;

    % We will read the transmitter's maximum I/Q buffer length and assign that value to a temporary variable.
    % NOTE:  We assume that the buffers sizes are the same for all interfaces
    % 读取发送器的最大I/Q缓冲区长度，并将该值分配给临时变量。 注意：假设所有接口的缓冲区大小相同
    maximum_buffer_len  = min(MAX_TX_LEN, wl_basebandCmd(node_tx, TX_RF_VEC, 'tx_buff_max_num_samples'));
    example_mode_string = 'hw';

else
    % 不使用Warp发送机的情况 [在sim-only版本(仿真)中使用与硬件相关的参数的合理默认值]
    maximum_buffer_len  = min(MAX_TX_LEN, 2^20);
    SAMP_FREQ           = 40e6;
    example_mode_string = 'sim';
end


%% -------------------------------------------------
%% 定义插值滤波器 
% 定义 半带滤波器的2^N倍数字内插 滤波器响应
interp_filt2 = zeros(1,43);
interp_filt2([1 3 5 7 9 11 13 15 17 19 21]) = [12 -32 72 -140 252 -422 682 -1086 1778 -3284 10364];
interp_filt2([23 25 27 29 31 33 35 37 39 41 43]) = interp_filt2(fliplr([1 3 5 7 9 11 13 15 17 19 21]));
interp_filt2(22) = 16384;
interp_filt2 = interp_filt2./max(abs(interp_filt2)); % 实现interp_filt2除以16384
% 定义了一个插值滤波器，具体数值可以在工作区查询


%% 定义前导码  
% 前导码中的短训练序列STS符号满足接收器处AGC核心所需的要求
% *** 我不清楚sts_f是从什么协议中定义的？ 是warp自定义的吗？
% 20190102 sts_f应该是802.11a协议定义的

% STS 短训练序列
sts_f = zeros(1,64);
sts_f(1:27)  = [0 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0];
sts_f(39:64) = [0 0 1+1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 ];
sts_t = ifft(sqrt(13/6).*sts_f, 64);  % 傅里叶逆变换，变换到时域中的前导码
sts_t = sts_t(1:16);                  % 只取16位的原因是，后面48位是前16位的重复

% 定义为了CFO的长训练序列LTS + 信道估计 
% LTS长训练序列long training sequence
% 载波频率偏移CFO 发送器的中心频率将不会完美地匹配接收器的中心频率
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64);

% Use 30 copies of the 16-sample STS for extra AGC settling margin 
% 前导码的具体生成设计：使用30份 sts_t 获得额外的AGC结算余量
preamble = [repmat(sts_t, 1, 30)  lts_t(33:64) lts_t lts_t]; 
% OFDM前导码16us：(10个符号的STS + 防护间隔 + 两个LTS) From:802.11无线网络权威指南P302
% 前导码中LTS的长度：160位 （64个样本LTS的2.5个副本）

% Sanity check variables that affect the number of Tx samples
% 影响Tx样本数量的完整性检查变量
num_samps_needed = ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)) + INTERP_RATE*((N_OFDM_SYMS * (N_SC + CP_LEN)) + length(preamble) +  ceil(length(interp_filt2)/2));                                
if(num_samps_needed > maximum_buffer_len)
    fprintf('Too many OFDM symbols for TX_NUM_SAMPS!\n');
    fprintf('Raise MAX_TX_LEN to %d, or \n', num_samps_needed);
    fprintf('Reduce N_OFDM_SYMS to %d\n', floor(((maximum_buffer_len - ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))/INTERP_RATE - (length(preamble) +  ceil(length(interp_filt2)/2)))/(N_SC + CP_LEN)));
    return;
end


%% 生成发送数据1
    % Generate a payload of random integers 生成随机整数的有效负载
    % 如果需要进行信源编码，信道编码，交织，那么就在通过其他函数生成合适的数据，然后发送机发送即可
rng(1); % 生成可预测的数字序列, - 使用非负整数 seed 为随机数生成函数提供种子
tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1; 
% MOD_ORDER 调制指数2/4/8/16; 从[0,15]随机生成一个数,这里一共生成N_DATA_SYMS个数据

%% #修改代码部分
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (N_SC_IND_DATA ~= 48)
    % Add_SC_OFDM = ceil(N_DATA_SYMS/N_SC_IND_DATA) - N_OFDM_SYMS; % 每一个子载波上添加的OFDM个数
    Add_OFDM = ceil(N_DATA_SYMS/N_SC_IND_DATA) * N_SC_IND_DATA - N_DATA_SYMS;  % 需要添加的OFDM个数，补0
    tx_data(N_DATA_SYMS+1 : N_DATA_SYMS+Add_OFDM) = 0;
    
    % 重新分配子载波之后，更新变量N_OFDM_SYMS，N_DATA_SYMS
    N_OFDM_SYMS = ceil(N_DATA_SYMS/N_SC_IND_DATA); % 更新每个子载波上传输OFDM的个数
    N_DATA_SYMS = length(tx_data); % 更新一共需要传输OFDM的个数
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 生成发送数据2
% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)  
% 像qam mod一样，避免了comm工具箱的要求
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8 
% 这些匿名函数实现了IEEE 802.11-2012 Section 18.3.5.8的调制映射
modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
modvec_64qam  =  (1/sqrt(43)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));
mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));

% 将数据值映射到复数符号
switch MOD_ORDER
    case 2         % BPSK
        tx_syms = arrayfun(mod_fcn_bpsk, tx_data);
    case 4         % QPSK
        tx_syms = arrayfun(mod_fcn_qpsk, tx_data);
    case 16        % 16-QAM
        tx_syms = arrayfun(mod_fcn_16qam, tx_data);
    case 64        % 64-QAM
        tx_syms = arrayfun(mod_fcn_64qam, tx_data);
    otherwise
        fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16, 64]\n', MOD_ORDER);
        return;
end

% Reshape the symbol vector to a matrix with one column per OFDM symbol 
% 将编码之后的符号tx_syms(星座图映射之后的数据) 分配到"每一个"子载波上
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS);  % 把生成的一维符号向量 排序为 (48*40)的矩阵

% Define the pilot tone values as BPSK symbols 将导频信号定义为BPSK符号 这里我觉得是兼容802.11b无法识别的原因
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols 在所有OFDM符号上重复导频
pilots_mat = repmat(pilots, 1, N_OFDM_SYMS); % 基于pilots矩阵进行扩展，生成了4*64矩阵



%% 傅里叶逆变换 IFFT
ifft_in_mat = zeros(N_SC, N_OFDM_SYMS); % (子载波数量64  OFDM符号数量40)
ifft_in_mat(SC_IND_DATA, :)   = tx_syms_mat;
ifft_in_mat(SC_IND_PILOTS, :) = pilots_mat;

tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);  % 这里对ifft_in_mat一个64*40的矩阵执行N_SC即64点的一维的逆傅里叶变换

if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end- CP_LEN + 1 : end), :); % 每一个OFDM符号，取出待发送数据后CP_LEN个子载波数据
    tx_payload_mat = [tx_cp; tx_payload_mat]; % 添加到待发送矩阵的前，添加了CP_LEN行
end


tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat)); % numel函数获取矩阵所有元素的个数  reshape让tx_payload_mat排成一行
tx_vec = [preamble tx_payload_vec];
tx_vec_padded = [tx_vec, zeros(1, ceil(length(interp_filt2)/2))]; % 在tx_vec后面添加了22列的0   tx_vec_padded是插值(0)之后的待发送数据

%% 插值 Interpolate 
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

tx_vec_2x = zeros(1, 2*numel(tx_vec_padded));
tx_vec_2x(1:2:end) = tx_vec_padded;
tx_vec_air = filter(interp_filt2, 1, tx_vec_2x); % 上采样与插值滤波器

tx_vec_air = TX_SCALE .* tx_vec_air ./ max(abs(tx_vec_air));
TX_NUM_SAMPS = length(tx_vec_air);

if(USE_WARPLAB_TXRX)
    wl_basebandCmd(nodes, 'tx_delay', 0);
    wl_basebandCmd(nodes, 'tx_length', TX_NUM_SAMPS);                                                        % Number of samples to send
    wl_basebandCmd(nodes, 'rx_length', TX_NUM_SAMPS + ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)));   % Number of samples to receive
end


%% ------Send-------------------------------------------
%% 使用Warp进行发送的过程
if(USE_WARPLAB_TXRX)
    wl_basebandCmd(node_tx, TX_RF_VEC, 'write_IQ', tx_vec_air(:));
    wl_interfaceCmd(node_tx, TX_RF, 'tx_en');
    wl_interfaceCmd(node_rx, RX_RF, 'rx_en');
    wl_basebandCmd(node_tx, TX_RF, 'tx_buff_en');
    wl_basebandCmd(node_rx, RX_RF, 'rx_buff_en');
    eth_trig.send();
    rx_vec_air = wl_basebandCmd(node_rx, RX_RF_VEC, 'read_IQ', 0, TX_NUM_SAMPS + (ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ))));
    rx_vec_air = rx_vec_air(:).';
    rx_vec_air_backup = rx_vec_air; % 备份rx_vec_air
    wl_basebandCmd(node_tx, TX_RF_ALL, 'tx_rx_buff_dis');
    wl_basebandCmd(node_rx, RX_RF_ALL, 'tx_rx_buff_dis');   
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_rx_dis');
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'tx_rx_dis');

else
    rx_vec_air = [tx_vec_air, zeros(1,ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))];
    rx_vec_air = rx_vec_air + 0*complex(randn(1,length(rx_vec_air)), randn(1,length(rx_vec_air)));
end


%% ------分析接收的数据----------------------------------------------
%% 抽样滤波 获得原始数据
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

raw_rx_dec = filter(interp_filt2, 1, rx_vec_air);
raw_rx_dec = raw_rx_dec(1:2:end); % get抽样之后的原始接收数据 —> raw_rx_dec



%% 关联长训练序列LTS
lts_corr = abs(conv(conj(fliplr(lts_t)), sign(raw_rx_dec)));  % Rx波形与时域LTS的复相关 sign是符号函数conv是卷积算法
lts_corr = lts_corr(32:end-32); % 跳过早期和晚期样本 - 避免AGC前样本偶尔出现误报
lts_peaks = find(lts_corr(1:800) > LTS_CORR_THRESH*max(lts_corr)); % 找到所有相关峰值correlation peaks % LTS_CORR_THRESH：LTS相关性的归一化阈值 本程序取值0.8
[LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks); % 选择最佳候选相关峰值作为LTS-有效载荷边界  % meshgrid作用：生成lts_peaks*lts_peaks 大矩阵
[lts_second_peak_index,y] = find(LTS2 - LTS1 == length(lts_t)); 

if(isempty(lts_second_peak_index))
    fprintf('No LTS Correlation Peaks Found!\n');
    return;
end

payload_ind = lts_peaks(max(lts_second_peak_index)) + 32;
lts_ind = payload_ind - 160;

if(DO_APPLY_CFO_CORRECTION) % 启用CFO估算/更正 Enable CFO estimation/correction
    % Extract LTS (not yet CFO corrected) 提取LTS（尚未更正CFO）
    rx_lts  = raw_rx_dec(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64 + -FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

    rx_cfo_est_lts = mean(unwrap(angle(rx_lts2 .* conj(rx_lts1))));
    rx_cfo_est_lts = rx_cfo_est_lts/(2*pi*64);
else
    rx_cfo_est_lts = 0;
end

rx_cfo_corr_t = exp(-1i*2*pi*rx_cfo_est_lts*[0:length(raw_rx_dec)-1]);
rx_dec_cfo_corr = raw_rx_dec .* rx_cfo_corr_t;


%% 信道估计
rx_lts  = rx_dec_cfo_corr(lts_ind : lts_ind+159);
rx_lts1 = rx_lts(-64 + -FFT_OFFSET + [97:160]);
rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);
rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f)/2;


%% 处理接收到的有效负载
payload_vec = rx_dec_cfo_corr(payload_ind : payload_ind + N_OFDM_SYMS*(N_SC+CP_LEN)-1); % 获得有效负载（前导码后的OFDM符号的整数） (标记位+发送OFDM数量*(子载波数量+循环前缀)-1)
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYMS); % 重排序有效负载 得到矩阵(子载波 + 循环前缀)*子载波数量 
payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);  % 删除循环前缀，保留CP的FFT_OFFSET样本（平均），其实这里还考虑了FFT_offset = 4,没有直接删除CP
syms_f_mat  = fft(payload_mat_noCP, N_SC, 1); % Take the FFT
syms_eq_mat = syms_f_mat ./ repmat(rx_H_est.', 1, N_OFDM_SYMS); % 均衡（迫零，只是除以信道估计） Why均衡需要这么做呢？ 20181123


if DO_APPLY_SFO_CORRECTION  % 启用SFO估计/更正  [用导频信号计算每个OFDM符号处的这个相位斜率，并使用该斜率为每个数据承载子载波内插相位校正]
    pilots_f_mat      = syms_eq_mat(SC_IND_PILOTS, :); % 提取均衡值中的导频
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat;  % 基于pilots扩展的pilots_mat 4*64的矩阵
    pilot_phases  = unwrap(angle(fftshift(pilots_f_mat_comp,1)), [], 1); % 计算每个Rx导频的相位   
    pilot_spacing_mat = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYMS);     % 计算每个OFDM符号中导频符号相位与频率的斜率                    
	pilot_slope_mat   = mean(diff(pilot_phases) ./ pilot_spacing_mat);   
    pilot_phase_sfo_corr = fftshift((-32:31).' * pilot_slope_mat, 1); % 计算每个OFDM符号的SFO校正相位
    pilot_phase_corr     = exp(-1i*(pilot_phase_sfo_corr));   
    syms_eq_mat = syms_eq_mat .* pilot_phase_corr; % 对每个符号 使用 导频相位校正
else
    pilot_phase_sfo_corr = zeros(N_SC, N_OFDM_SYMS); % 定义一个空的SFO校正矩阵（由下面的绘图代码使用）
end


if DO_APPLY_PHASE_ERR_CORRECTION   % 启用剩余CFO估算/更正  [提取导频并计算每符号相位误差]
    pilots_f_mat      = syms_eq_mat(SC_IND_PILOTS, :); 
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat;
    pilot_phase_err   = angle(mean(pilots_f_mat_comp));
else
   pilot_phase_err = zeros(1, N_OFDM_SYMS); % 定义空相位校正向量（由下面的绘图代码使用）
end


pilot_phase_err_corr = repmat(pilot_phase_err, N_SC, 1);
pilot_phase_corr     = exp(-1i*(pilot_phase_err_corr));

syms_eq_pc_mat   = syms_eq_mat .* pilot_phase_corr; % 对每个符号应用导频相位校正
payload_syms_mat = syms_eq_pc_mat(SC_IND_DATA, :);



%% 解调
rx_syms = reshape(payload_syms_mat, 1, N_DATA_SYMS);
demod_fcn_bpsk  = @(x) double(real(x)>0);
demod_fcn_qpsk  = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data = arrayfun(demod_fcn_bpsk, rx_syms);
    case 4         % QPSK
        rx_data = arrayfun(demod_fcn_qpsk, rx_syms);
    case 16        % 16-QAM
        rx_data = arrayfun(demod_fcn_16qam, rx_syms);
    case 64        % 64-QAM
        rx_data = arrayfun(demod_fcn_64qam, rx_syms);
end

%% SER分析
s0 = 0; % 临时计数器，记录错误个数 
for i = 1:N_DATA_SYMS_Copy % N_DATA_SYMS是实际发送总的OFDM数量   
    if( tx_data(i) ~= rx_data(i) )
        s0  =  s0 + 1;  
    end   
end
s1 = 1 - s0 / N_DATA_SYMS_Copy % SER值
eval(['save ' 'A' num2str(M1) '.mat']); % 实验保存工作区的所有数据，以A1,A2...保存
end

fprintf('Congratulations, the experiment is over!\n');