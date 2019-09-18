clc;
clear;  
close all;

for M1 = 1:100 % ѭ������
%% ��������
% Params:
USE_WARPLAB_TXRX = 1;   % Enable WARPLab-in-the-loop (otherwise sim-only) ����WARPLab-in-the-loop���������sim��
WRITE_PNG_FILES  = 0;   % Enable writing plots to PNG  ���ý�ͼ��д��PNG
CHANNEL          = 11;  % Channel to tune Tx and Rx radios ѡ���ͻ��ͽ��ջ���ʹ�õ��ŵ�

N_OFDM_SYMS      = 40;  % Number of OFDM symbols OFDM����ʵ�ʷ��͵��ź�����
MOD_ORDER        = 16;  % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM) ���õĵ��Ʒ�ʽ
TX_SCALE         = 1.0; % Scale for Tx waveform ([0:1]) ����Tx����
SC_IND_PILOTS  = [8 22 44 58];                        % Pilot subcarrier indices ��Ƶ���ز����
SC_IND_DATA    = [2:7 9:21 23:27 39:43 45:57 59:64];  % Data subcarrier indices  �������ز����
N_SC_IND_DATA  = length(SC_IND_DATA); 
N_SC           = 64;                                  % Number of subcarriers ���ز�����
CP_LEN         = 16;                                  % Cyclic prefix length  OFDMѭ��ǰ׺�ĳ���
N_DATA_SYMS    = N_OFDM_SYMS * 48;                    % @����OFDM���ݷ�����
N_DATA_SYMS_Copy = N_DATA_SYMS;                       % ����N_DATA_SYMS������
INTERP_RATE    = 2;                                   % Interpolation rate (must be 2) ��ֵ�ʣ�����Ϊ2��
FFT_OFFSET                    = 4;           % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation LTS����ԵĹ�һ����ֵ
DO_APPLY_CFO_CORRECTION       = 1;           % Enable CFO estimation/correction ����CFO����/����
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction ����ʣ��CFO����/����
DO_APPLY_SFO_CORRECTION       = 1;           % Enable SFO estimation/correction ����SFO����/����
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

    % Get parameters from the node �ӽڵ��ȡ���� - ����Ƶ��
    SAMP_FREQ = wl_basebandCmd(nodes(1), 'tx_buff_clk_freq'); % ? *SAMP_FREQ��ָ����Ƶ����
    Ts        = 1/SAMP_FREQ;

    % We will read the transmitter's maximum I/Q buffer length and assign that value to a temporary variable.
    % NOTE:  We assume that the buffers sizes are the same for all interfaces
    % ��ȡ�����������I/Q���������ȣ�������ֵ�������ʱ������ ע�⣺�������нӿڵĻ�������С��ͬ
    maximum_buffer_len  = min(MAX_TX_LEN, wl_basebandCmd(node_tx, TX_RF_VEC, 'tx_buff_max_num_samples'));
    example_mode_string = 'hw';

else
    % ��ʹ��Warp���ͻ������ [��sim-only�汾(����)��ʹ����Ӳ����صĲ����ĺ���Ĭ��ֵ]
    maximum_buffer_len  = min(MAX_TX_LEN, 2^20);
    SAMP_FREQ           = 40e6;
    example_mode_string = 'sim';
end


%% -------------------------------------------------
%% �����ֵ�˲��� 
% ���� ����˲�����2^N�������ڲ� �˲�����Ӧ
interp_filt2 = zeros(1,43);
interp_filt2([1 3 5 7 9 11 13 15 17 19 21]) = [12 -32 72 -140 252 -422 682 -1086 1778 -3284 10364];
interp_filt2([23 25 27 29 31 33 35 37 39 41 43]) = interp_filt2(fliplr([1 3 5 7 9 11 13 15 17 19 21]));
interp_filt2(22) = 16384;
interp_filt2 = interp_filt2./max(abs(interp_filt2)); % ʵ��interp_filt2����16384
% ������һ����ֵ�˲�����������ֵ�����ڹ�������ѯ


%% ����ǰ����  
% ǰ�����еĶ�ѵ������STS���������������AGC���������Ҫ��
% *** �Ҳ����sts_f�Ǵ�ʲôЭ���ж���ģ� ��warp�Զ������
% 20190102 sts_fӦ����802.11aЭ�鶨���

% STS ��ѵ������
sts_f = zeros(1,64);
sts_f(1:27)  = [0 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0];
sts_f(39:64) = [0 0 1+1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 ];
sts_t = ifft(sqrt(13/6).*sts_f, 64);  % ����Ҷ��任���任��ʱ���е�ǰ����
sts_t = sts_t(1:16);                  % ֻȡ16λ��ԭ���ǣ�����48λ��ǰ16λ���ظ�

% ����Ϊ��CFO�ĳ�ѵ������LTS + �ŵ����� 
% LTS��ѵ������long training sequence
% �ز�Ƶ��ƫ��CFO ������������Ƶ�ʽ�����������ƥ�������������Ƶ��
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64);

% Use 30 copies of the 16-sample STS for extra AGC settling margin 
% ǰ����ľ���������ƣ�ʹ��30�� sts_t ��ö����AGC��������
preamble = [repmat(sts_t, 1, 30)  lts_t(33:64) lts_t lts_t]; 
% OFDMǰ����16us��(10�����ŵ�STS + ������� + ����LTS) From:802.11��������Ȩ��ָ��P302
% ǰ������LTS�ĳ��ȣ�160λ ��64������LTS��2.5��������

% Sanity check variables that affect the number of Tx samples
% Ӱ��Tx���������������Լ�����
num_samps_needed = ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)) + INTERP_RATE*((N_OFDM_SYMS * (N_SC + CP_LEN)) + length(preamble) +  ceil(length(interp_filt2)/2));                                
if(num_samps_needed > maximum_buffer_len)
    fprintf('Too many OFDM symbols for TX_NUM_SAMPS!\n');
    fprintf('Raise MAX_TX_LEN to %d, or \n', num_samps_needed);
    fprintf('Reduce N_OFDM_SYMS to %d\n', floor(((maximum_buffer_len - ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))/INTERP_RATE - (length(preamble) +  ceil(length(interp_filt2)/2)))/(N_SC + CP_LEN)));
    return;
end


%% ���ɷ�������1
    % Generate a payload of random integers ���������������Ч����
    % �����Ҫ������Դ���룬�ŵ����룬��֯����ô����ͨ�������������ɺ��ʵ����ݣ�Ȼ���ͻ����ͼ���
rng(1); % ���ɿ�Ԥ�����������, - ʹ�÷Ǹ����� seed Ϊ��������ɺ����ṩ����
tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1; 
% MOD_ORDER ����ָ��2/4/8/16; ��[0,15]�������һ����,����һ������N_DATA_SYMS������

%% #�޸Ĵ��벿��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (N_SC_IND_DATA ~= 48)
    % Add_SC_OFDM = ceil(N_DATA_SYMS/N_SC_IND_DATA) - N_OFDM_SYMS; % ÿһ�����ز�����ӵ�OFDM����
    Add_OFDM = ceil(N_DATA_SYMS/N_SC_IND_DATA) * N_SC_IND_DATA - N_DATA_SYMS;  % ��Ҫ��ӵ�OFDM��������0
    tx_data(N_DATA_SYMS+1 : N_DATA_SYMS+Add_OFDM) = 0;
    
    % ���·������ز�֮�󣬸��±���N_OFDM_SYMS��N_DATA_SYMS
    N_OFDM_SYMS = ceil(N_DATA_SYMS/N_SC_IND_DATA); % ����ÿ�����ز��ϴ���OFDM�ĸ���
    N_DATA_SYMS = length(tx_data); % ����һ����Ҫ����OFDM�ĸ���
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ���ɷ�������2
% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)  
% ��qam modһ����������comm�������Ҫ��
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8 
% ��Щ��������ʵ����IEEE 802.11-2012 Section 18.3.5.8�ĵ���ӳ��
modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
modvec_64qam  =  (1/sqrt(43)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));
mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));

% ������ֵӳ�䵽��������
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
% ������֮��ķ���tx_syms(����ͼӳ��֮�������) ���䵽"ÿһ��"���ز���
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS);  % �����ɵ�һά�������� ����Ϊ (48*40)�ľ���

% Define the pilot tone values as BPSK symbols ����Ƶ�źŶ���ΪBPSK���� �����Ҿ����Ǽ���802.11b�޷�ʶ���ԭ��
pilots = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols ������OFDM�������ظ���Ƶ
pilots_mat = repmat(pilots, 1, N_OFDM_SYMS); % ����pilots���������չ��������4*64����



%% ����Ҷ��任 IFFT
ifft_in_mat = zeros(N_SC, N_OFDM_SYMS); % (���ز�����64  OFDM��������40)
ifft_in_mat(SC_IND_DATA, :)   = tx_syms_mat;
ifft_in_mat(SC_IND_PILOTS, :) = pilots_mat;

tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);  % �����ifft_in_matһ��64*40�ľ���ִ��N_SC��64���һά���渵��Ҷ�任

if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end- CP_LEN + 1 : end), :); % ÿһ��OFDM���ţ�ȡ�����������ݺ�CP_LEN�����ز�����
    tx_payload_mat = [tx_cp; tx_payload_mat]; % ��ӵ������;����ǰ�������CP_LEN��
end


tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat)); % numel������ȡ��������Ԫ�صĸ���  reshape��tx_payload_mat�ų�һ��
tx_vec = [preamble tx_payload_vec];
tx_vec_padded = [tx_vec, zeros(1, ceil(length(interp_filt2)/2))]; % ��tx_vec���������22�е�0   tx_vec_padded�ǲ�ֵ(0)֮��Ĵ���������

%% ��ֵ Interpolate 
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

tx_vec_2x = zeros(1, 2*numel(tx_vec_padded));
tx_vec_2x(1:2:end) = tx_vec_padded;
tx_vec_air = filter(interp_filt2, 1, tx_vec_2x); % �ϲ������ֵ�˲���

tx_vec_air = TX_SCALE .* tx_vec_air ./ max(abs(tx_vec_air));
TX_NUM_SAMPS = length(tx_vec_air);

if(USE_WARPLAB_TXRX)
    wl_basebandCmd(nodes, 'tx_delay', 0);
    wl_basebandCmd(nodes, 'tx_length', TX_NUM_SAMPS);                                                        % Number of samples to send
    wl_basebandCmd(nodes, 'rx_length', TX_NUM_SAMPS + ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)));   % Number of samples to receive
end


%% ------Send-------------------------------------------
%% ʹ��Warp���з��͵Ĺ���
if(USE_WARPLAB_TXRX)
    wl_basebandCmd(node_tx, TX_RF_VEC, 'write_IQ', tx_vec_air(:));
    wl_interfaceCmd(node_tx, TX_RF, 'tx_en');
    wl_interfaceCmd(node_rx, RX_RF, 'rx_en');
    wl_basebandCmd(node_tx, TX_RF, 'tx_buff_en');
    wl_basebandCmd(node_rx, RX_RF, 'rx_buff_en');
    eth_trig.send();
    rx_vec_air = wl_basebandCmd(node_rx, RX_RF_VEC, 'read_IQ', 0, TX_NUM_SAMPS + (ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ))));
    rx_vec_air = rx_vec_air(:).';
    rx_vec_air_backup = rx_vec_air; % ����rx_vec_air
    wl_basebandCmd(node_tx, TX_RF_ALL, 'tx_rx_buff_dis');
    wl_basebandCmd(node_rx, RX_RF_ALL, 'tx_rx_buff_dis');   
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_rx_dis');
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'tx_rx_dis');

else
    rx_vec_air = [tx_vec_air, zeros(1,ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))];
    rx_vec_air = rx_vec_air + 0*complex(randn(1,length(rx_vec_air)), randn(1,length(rx_vec_air)));
end


%% ------�������յ�����----------------------------------------------
%% �����˲� ���ԭʼ����
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

raw_rx_dec = filter(interp_filt2, 1, rx_vec_air);
raw_rx_dec = raw_rx_dec(1:2:end); % get����֮���ԭʼ�������� ��> raw_rx_dec



%% ������ѵ������LTS
lts_corr = abs(conv(conj(fliplr(lts_t)), sign(raw_rx_dec)));  % Rx������ʱ��LTS�ĸ���� sign�Ƿ��ź���conv�Ǿ���㷨
lts_corr = lts_corr(32:end-32); % �������ں��������� - ����AGCǰ����ż��������
lts_peaks = find(lts_corr(1:800) > LTS_CORR_THRESH*max(lts_corr)); % �ҵ�������ط�ֵcorrelation peaks % LTS_CORR_THRESH��LTS����ԵĹ�һ����ֵ ������ȡֵ0.8
[LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks); % ѡ����Ѻ�ѡ��ط�ֵ��ΪLTS-��Ч�غɱ߽�  % meshgrid���ã�����lts_peaks*lts_peaks �����
[lts_second_peak_index,y] = find(LTS2 - LTS1 == length(lts_t)); 

if(isempty(lts_second_peak_index))
    fprintf('No LTS Correlation Peaks Found!\n');
    return;
end

payload_ind = lts_peaks(max(lts_second_peak_index)) + 32;
lts_ind = payload_ind - 160;

if(DO_APPLY_CFO_CORRECTION) % ����CFO����/���� Enable CFO estimation/correction
    % Extract LTS (not yet CFO corrected) ��ȡLTS����δ����CFO��
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


%% �ŵ�����
rx_lts  = rx_dec_cfo_corr(lts_ind : lts_ind+159);
rx_lts1 = rx_lts(-64 + -FFT_OFFSET + [97:160]);
rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);
rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f)/2;


%% ������յ�����Ч����
payload_vec = rx_dec_cfo_corr(payload_ind : payload_ind + N_OFDM_SYMS*(N_SC+CP_LEN)-1); % �����Ч���أ�ǰ������OFDM���ŵ������� (���λ+����OFDM����*(���ز�����+ѭ��ǰ׺)-1)
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYMS); % ��������Ч���� �õ�����(���ز� + ѭ��ǰ׺)*���ز����� 
payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);  % ɾ��ѭ��ǰ׺������CP��FFT_OFFSET������ƽ��������ʵ���ﻹ������FFT_offset = 4,û��ֱ��ɾ��CP
syms_f_mat  = fft(payload_mat_noCP, N_SC, 1); % Take the FFT
syms_eq_mat = syms_f_mat ./ repmat(rx_H_est.', 1, N_OFDM_SYMS); % ���⣨���㣬ֻ�ǳ����ŵ����ƣ� Why������Ҫ��ô���أ� 20181123


if DO_APPLY_SFO_CORRECTION  % ����SFO����/����  [�õ�Ƶ�źż���ÿ��OFDM���Ŵ��������λб�ʣ���ʹ�ø�б��Ϊÿ�����ݳ������ز��ڲ���λУ��]
    pilots_f_mat      = syms_eq_mat(SC_IND_PILOTS, :); % ��ȡ����ֵ�еĵ�Ƶ
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat;  % ����pilots��չ��pilots_mat 4*64�ľ���
    pilot_phases  = unwrap(angle(fftshift(pilots_f_mat_comp,1)), [], 1); % ����ÿ��Rx��Ƶ����λ   
    pilot_spacing_mat = repmat(mod(diff(fftshift(SC_IND_PILOTS)),64).', 1, N_OFDM_SYMS);     % ����ÿ��OFDM�����е�Ƶ������λ��Ƶ�ʵ�б��                    
	pilot_slope_mat   = mean(diff(pilot_phases) ./ pilot_spacing_mat);   
    pilot_phase_sfo_corr = fftshift((-32:31).' * pilot_slope_mat, 1); % ����ÿ��OFDM���ŵ�SFOУ����λ
    pilot_phase_corr     = exp(-1i*(pilot_phase_sfo_corr));   
    syms_eq_mat = syms_eq_mat .* pilot_phase_corr; % ��ÿ������ ʹ�� ��Ƶ��λУ��
else
    pilot_phase_sfo_corr = zeros(N_SC, N_OFDM_SYMS); % ����һ���յ�SFOУ������������Ļ�ͼ����ʹ�ã�
end


if DO_APPLY_PHASE_ERR_CORRECTION   % ����ʣ��CFO����/����  [��ȡ��Ƶ������ÿ������λ���]
    pilots_f_mat      = syms_eq_mat(SC_IND_PILOTS, :); 
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat;
    pilot_phase_err   = angle(mean(pilots_f_mat_comp));
else
   pilot_phase_err = zeros(1, N_OFDM_SYMS); % �������λУ��������������Ļ�ͼ����ʹ�ã�
end


pilot_phase_err_corr = repmat(pilot_phase_err, N_SC, 1);
pilot_phase_corr     = exp(-1i*(pilot_phase_err_corr));

syms_eq_pc_mat   = syms_eq_mat .* pilot_phase_corr; % ��ÿ������Ӧ�õ�Ƶ��λУ��
payload_syms_mat = syms_eq_pc_mat(SC_IND_DATA, :);



%% ���
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

%% SER����
s0 = 0; % ��ʱ����������¼������� 
for i = 1:N_DATA_SYMS_Copy % N_DATA_SYMS��ʵ�ʷ����ܵ�OFDM����   
    if( tx_data(i) ~= rx_data(i) )
        s0  =  s0 + 1;  
    end   
end
s1 = 1 - s0 / N_DATA_SYMS_Copy % SERֵ
eval(['save ' 'A' num2str(M1) '.mat']); % ʵ�鱣�湤�������������ݣ���A1,A2...����
end

fprintf('Congratulations, the experiment is over!\n');