clc; clear; close all;
modelName      = 'V2V_Simulation';
simTime        = 10;        % seconds
initVelA       = 20;       % m/s — Vehicle A initial velocity
initVelB       = 20;       % m/s — Vehicle B initial velocity
brakeAccelA    = -7;      % m/s² — Vehicle A braking deceleration
brakeAccelB    = -6;      % m/s² — Vehicle B braking deceleration
obstacleThresh = 15;        % metres — alert threshold
initGap_AB     = 15;       % metres — initial gap B behind A
initDist_AO    = 80;       % metres — initial A-to-obstacle distance
brakeStartTime = 2.5;        % seconds — when Vehicle A begins braking
reactionDelay  = 0.2;      % seconds — 200 ms V2V transport delay

% Architecture:
%   Vehicle A starts at x=0, velocity=20 m/s
%   Vehicle B starts at x=-15, velocity=20 m/s (15m behind A)
%   Obstacle fixed at x=50 m

fprintf('\n╔══════════════════════════════════════════════════════╗\n');
fprintf('║  V2V SAFETY SYSTEM — Collision Avoidance Simulator    ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');
fprintf('📋 Configuration:\n');
fprintf('   • Simulation time:     %.1f s\n', simTime);
fprintf('   • Initial velocities:  %.1f m/s (both)\n', initVelA);
fprintf('   • Brake acceleration:  A=%.1f, B=%.1f m/s²\n', brakeAccelA, brakeAccelB);
fprintf('   • Initial gap A→B:     %.1f m\n', initGap_AB);
fprintf('   • Obstacle distance:   %.1f m\n', initDist_AO);
fprintf('   • Alert threshold:     %.1f m\n', obstacleThresh);
fprintf('   • V2V delay:           %.0f ms\n\n', reactionDelay*1000);

%% ── 2. CREATE / RELOAD MODEL ────────────────────────────────
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Convenience path builder
P = @(name) [modelName '/' name];

fprintf('🔧 Building Simulink model...\n');

%% ── 3. SOLVER CONFIGURATION ─────────────────────────────────
set_param(modelName, ...
    'Solver',         'ode45',          ...
    'StopTime',       num2str(simTime), ...
    'MaxStep',        '0.01',           ...
    'SolverType',     'Variable-step');

%% ── 4. BLOCK POSITIONS ──────────────────────────────────────
% [left top right bottom] in pixels

% ══════════════════════════════════════════════════════════════
% VEHICLE A (TOP SECTION)
% ══════════════════════════════════════════════════════════════

% Row 1: Brake trigger (y=60-90)
pos.stepA        = [30   60   90   90];
pos.gainBrakeA   = [130  60   190  90];
pos.constZeroA   = [90   110  150  140];
pos.switchA      = [230  35   270  125];

% Row 2: Velocity integration (y=60-110)
pos.intVelA      = [310  60   370  110];
pos.satVelA      = [410  60   470  110];

% Row 3: Position integration (y=160-210)
pos.intPosA      = [410  160  470  210];

% ══════════════════════════════════════════════════════════════
% OBSTACLE DETECTION (MIDDLE SECTION)
% ══════════════════════════════════════════════════════════════

% Distance calculation (y=260-310)
pos.constObst    = [30   270  90   300];
pos.sumDistAO    = [150  255  200  315];
pos.satDistAO    = [240  270  300  310];

% Alert logic (y=260-300)
pos.relOpAO      = [340  260  410  300];
pos.constThresh  = [260  330  320  360];
pos.bool2dbl     = [450  260  510  300];

% Communication (y=260-300)
pos.alertDelay   = [550  260  630  300];

% Driver response logic (y=340-400)
pos.constManual  = [460  360  520  390];
pos.notResp      = [560  355  620  385];
pos.andGate      = [660  315  720  385];
pos.bool2dblB    = [760  330  820  370];

% Auto-brake gain (y=330-370)
pos.autoBrakeGain= [860  330  920  370];

% ══════════════════════════════════════════════════════════════
% VEHICLE B (BOTTOM SECTION)
% ══════════════════════════════════════════════════════════════

% Brake switch (y=440-530)
pos.constZeroB   = [860  490  920  520];
pos.switchB      = [960  415  1000 505];

% Velocity integration (y=440-490)
pos.intVelB      = [1040 440  1100 490];
pos.satVelB      = [1140 440  1200 490];

% Position integration (y=560-610)
pos.intPosB      = [1140 560  1200 610];

% ══════════════════════════════════════════════════════════════
% DISTANCE A→B CALCULATION
% ══════════════════════════════════════════════════════════════
pos.sumDistAB    = [680  560  730  620];
pos.satDistAB    = [770  570  830  610];

% ══════════════════════════════════════════════════════════════
% SCOPES
% ══════════════════════════════════════════════════════════════
pos.muxVel       = [1260 60   1290 110];
pos.scopeVel     = [1330 55   1390 115];
pos.muxDist      = [900  270  930  320];
pos.scopeDist    = [970  265  1030 325];

fprintf('   ✓ Layout positions defined\n');

%% ── 5. BUILD VEHICLE A ──────────────────────────────────────

% ┌─────────────────────────────────────────────────────────┐
% │ VEHICLE A: BRAKE TRIGGER → VELOCITY → POSITION         │
% └─────────────────────────────────────────────────────────┘

% Step: outputs 0 before t=3s, 1 after (brake command)
add_block('simulink/Sources/Step', P('BrakeStep_A'), ...
    'Position', pos.stepA, ...
    'Time',     num2str(brakeStartTime), ...
    'Before',   '0', ...
    'After',    '1');

% Gain: multiply brake trigger by deceleration magnitude
add_block('simulink/Math Operations/Gain', P('GainBrakeA'), ...
    'Position', pos.gainBrakeA, ...
    'Gain',     num2str(brakeAccelA));  % -7

% Constant: zero acceleration (for "not braking" state)
add_block('simulink/Sources/Constant', P('ConstZeroAccelA'), ...
    'Position', pos.constZeroA, ...
    'Value',    '0');

% Switch: if BrakeStep > 0.5 → use brakeAccel, else 0
%   Port 1: if-true (brakeAccel)
%   Port 2: condition (BrakeStep)
%   Port 3: if-false (zero)
add_block('simulink/Signal Routing/Switch', P('SwitchAccelA'), ...
    'Position', pos.switchA, ...
    'Threshold', '0.5');

% Integrator: accel → velocity (IC = initial velocity)
add_block('simulink/Continuous/Integrator', P('IntVelA'), ...
    'Position',         pos.intVelA, ...
    'InitialCondition', num2str(initVelA));  % Starts at 20 m/s

% Saturation: clamp velocity ≥ 0 (can't go backward)
add_block('simulink/Discontinuities/Saturation', P('SatVelA'), ...
    'Position',    pos.satVelA, ...
    'UpperLimit',  '100', ...
    'LowerLimit',  '0');

% Integrator: velocity → position (IC = 0, starts at origin)
add_block('simulink/Continuous/Integrator', P('IntPosA'), ...
    'Position',         pos.intPosA, ...
    'InitialCondition', '0');

fprintf('   ✓ Vehicle A blocks created\n');

%% ── 6. BUILD OBSTACLE DETECTION ─────────────────────────────

% ┌─────────────────────────────────────────────────────────┐
% │ DISTANCE A→OBSTACLE = obstaclePos - posA                │
% └─────────────────────────────────────────────────────────┘

% Constant: obstacle position (fixed in world frame)
add_block('simulink/Sources/Constant', P('ConstObstPos'), ...
    'Position', pos.constObst, ...
    'Value',    num2str(initDist_AO));  % 50 m

% Sum: distAO = obstaclePos(+) - posA(-)
add_block('simulink/Math Operations/Sum', P('SumDistAO'), ...
    'Position', pos.sumDistAO, ...
    'Inputs',   '+-');

% Saturation: clamp distance ≥ 0 (collision floor)
add_block('simulink/Discontinuities/Saturation', P('SatDistAO'), ...
    'Position',    pos.satDistAO, ...
    'UpperLimit',  '200', ...
    'LowerLimit',  '0');

% ┌─────────────────────────────────────────────────────────┐
% │ ALERT LOGIC: distAO < threshold                         │
% └─────────────────────────────────────────────────────────┘

% Relational Operator: distAO < 5m → boolean alert
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    P('RelOpAO'), ...
    'Position', pos.relOpAO, ...
    'Operator', '<');

% Constant: alert threshold
add_block('simulink/Sources/Constant', P('ConstThresh'), ...
    'Position', pos.constThresh, ...
    'Value',    num2str(obstacleThresh));  % 5 m

% Data Type Conversion: boolean → double (required before Transport Delay)
add_block('simulink/Signal Attributes/Data Type Conversion', P('Bool2Dbl'), ...
    'Position',         pos.bool2dbl, ...
    'OutDataTypeStr',   'double');

% Transport Delay: V2V communication latency
add_block('simulink/Continuous/Transport Delay', P('AlertDelay'), ...
    'Position',     pos.alertDelay, ...
    'DelayTime',    num2str(reactionDelay), ...
    'InitialInput', '0');

fprintf('   ✓ Obstacle detection blocks created\n');

%% ── 7. BUILD DRIVER RESPONSE LOGIC ──────────────────────────

% ┌─────────────────────────────────────────────────────────┐
% │ AUTO-BRAKE CONDITION:                                   │
% │   (Alert received) AND (Driver hasn't responded)        │
% └─────────────────────────────────────────────────────────┘

% Constant: manual brake flag (0 = driver NOT braking manually)
add_block('simulink/Sources/Constant', P('ManualBrakeB'), ...
    'Position', pos.constManual, ...
    'Value',    '0');

% NOT gate: invert manual brake (0→1, driver hasn't reacted)
add_block('simulink/Logic and Bit Operations/Logical Operator', P('NotResp'), ...
    'Position', pos.notResp, ...
    'Operator', 'NOT');

% AND gate: (delayed alert) AND (no manual brake)
add_block('simulink/Logic and Bit Operations/Logical Operator', P('AndGate'), ...
    'Position', pos.andGate, ...
    'Operator', 'AND', ...
    'Inputs',   '2');

% Data Type Conversion: boolean → double (before Gain)
add_block('simulink/Signal Attributes/Data Type Conversion', P('Bool2DblB'), ...
    'Position',       pos.bool2dblB, ...
    'OutDataTypeStr', 'double');

% Gain: multiply trigger (0 or 1) by brake deceleration
add_block('simulink/Math Operations/Gain', P('AutoBrakeGain'), ...
    'Position', pos.autoBrakeGain, ...
    'Gain',     num2str(brakeAccelB));  % -6

fprintf('   ✓ Driver response logic created\n');

%% ── 8. BUILD VEHICLE B ──────────────────────────────────────

% ┌─────────────────────────────────────────────────────────┐
% │ VEHICLE B: AUTO-BRAKE → VELOCITY → POSITION            │
% └─────────────────────────────────────────────────────────┘

% Constant: zero acceleration (for "not braking" state)
add_block('simulink/Sources/Constant', P('ConstZeroAccelB'), ...
    'Position', pos.constZeroB, ...
    'Value',    '0');

% Switch: if auto-brake trigger > 0.5 → use brakeAccel, else 0
add_block('simulink/Signal Routing/Switch', P('SwitchAccelB'), ...
    'Position', pos.switchB, ...
    'Threshold', '0.5');

% Integrator: accel → velocity (IC = initial velocity)
add_block('simulink/Continuous/Integrator', P('IntVelB'), ...
    'Position',         pos.intVelB, ...
    'InitialCondition', num2str(initVelB));  % 20 m/s

% Saturation: clamp velocity ≥ 0
add_block('simulink/Discontinuities/Saturation', P('SatVelB'), ...
    'Position',    pos.satVelB, ...
    'UpperLimit',  '100', ...
    'LowerLimit',  '0');

% Integrator: velocity → position (IC = -15 m, starts 15m behind A)
add_block('simulink/Continuous/Integrator', P('IntPosB'), ...
    'Position',         pos.intPosB, ...
    'InitialCondition', num2str(-initGap_AB));  % -15

fprintf('   ✓ Vehicle B blocks created\n');

%% ── 9. BUILD DISTANCE A→B ───────────────────────────────────

% ┌─────────────────────────────────────────────────────────┐
% │ DISTANCE A→B = posA - posB                              │
% │   At t=0: 0 - (-15) = 15 m ✓                            │
% └─────────────────────────────────────────────────────────┘

% Sum: distAB = posA(+) - posB(-)
add_block('simulink/Math Operations/Sum', P('SumDistAB'), ...
    'Position', pos.sumDistAB, ...
    'Inputs',   '+-');

% Saturation: clamp distance ≥ 0
add_block('simulink/Discontinuities/Saturation', P('SatDistAB'), ...
    'Position',    pos.satDistAB, ...
    'UpperLimit',  '200', ...
    'LowerLimit',  '0');

fprintf('   ✓ Distance calculation blocks created\n');

%% ── 10. BUILD SCOPES ────────────────────────────────────────

% Velocity Scope
add_block('simulink/Signal Routing/Mux', P('MuxVel'), ...
    'Position', pos.muxVel, 'Inputs', '2');

add_block('simulink/Sinks/Scope', P('Scope_Velocity'), ...
    'Position',      pos.scopeVel, ...
    'NumInputPorts', '1');

% Distance Scope
add_block('simulink/Signal Routing/Mux', P('MuxDist'), ...
    'Position', pos.muxDist, 'Inputs', '2');

add_block('simulink/Sinks/Scope', P('Scope_Distance'), ...
    'Position',      pos.scopeDist, ...
    'NumInputPorts', '1');

fprintf('   ✓ Scope blocks created\n');

%% ── 11. CONNECT ALL SIGNALS ─────────────────────────────────

fprintf('🔌 Connecting signals...\n');

% ═══════════════════════════════════════════════════════════
% VEHICLE A CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'BrakeStep_A/1',      'GainBrakeA/1',       'autorouting','on');
add_line(modelName, 'GainBrakeA/1',       'SwitchAccelA/1',     'autorouting','on');
add_line(modelName, 'BrakeStep_A/1',      'SwitchAccelA/2',     'autorouting','on');
add_line(modelName, 'ConstZeroAccelA/1',  'SwitchAccelA/3',     'autorouting','on');
add_line(modelName, 'SwitchAccelA/1',     'IntVelA/1',          'autorouting','on');
add_line(modelName, 'IntVelA/1',          'SatVelA/1',          'autorouting','on');
add_line(modelName, 'SatVelA/1',          'IntPosA/1',          'autorouting','on');

% ═══════════════════════════════════════════════════════════
% OBSTACLE DETECTION CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'ConstObstPos/1',     'SumDistAO/1',        'autorouting','on');
add_line(modelName, 'IntPosA/1',          'SumDistAO/2',        'autorouting','on');
add_line(modelName, 'SumDistAO/1',        'SatDistAO/1',        'autorouting','on');
add_line(modelName, 'SatDistAO/1',        'RelOpAO/1',          'autorouting','on');
add_line(modelName, 'ConstThresh/1',      'RelOpAO/2',          'autorouting','on');
add_line(modelName, 'RelOpAO/1',          'Bool2Dbl/1',         'autorouting','on');
add_line(modelName, 'Bool2Dbl/1',         'AlertDelay/1',       'autorouting','on');

% ═══════════════════════════════════════════════════════════
% DRIVER RESPONSE LOGIC CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'AlertDelay/1',       'AndGate/1',          'autorouting','on');
add_line(modelName, 'ManualBrakeB/1',     'NotResp/1',          'autorouting','on');
add_line(modelName, 'NotResp/1',          'AndGate/2',          'autorouting','on');
add_line(modelName, 'AndGate/1',          'Bool2DblB/1',        'autorouting','on');
add_line(modelName, 'Bool2DblB/1',        'AutoBrakeGain/1',    'autorouting','on');

% ═══════════════════════════════════════════════════════════
% VEHICLE B CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'AutoBrakeGain/1',    'SwitchAccelB/1',     'autorouting','on');
add_line(modelName, 'AndGate/1',          'SwitchAccelB/2',     'autorouting','on');
add_line(modelName, 'ConstZeroAccelB/1',  'SwitchAccelB/3',     'autorouting','on');
add_line(modelName, 'SwitchAccelB/1',     'IntVelB/1',          'autorouting','on');
add_line(modelName, 'IntVelB/1',          'SatVelB/1',          'autorouting','on');
add_line(modelName, 'SatVelB/1',          'IntPosB/1',          'autorouting','on');

% ═══════════════════════════════════════════════════════════
% DISTANCE A→B CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'IntPosA/1',          'SumDistAB/1',        'autorouting','on');
add_line(modelName, 'IntPosB/1',          'SumDistAB/2',        'autorouting','on');
add_line(modelName, 'SumDistAB/1',        'SatDistAB/1',        'autorouting','on');

% ═══════════════════════════════════════════════════════════
% SCOPE CONNECTIONS
% ═══════════════════════════════════════════════════════════
add_line(modelName, 'SatVelA/1',          'MuxVel/1',           'autorouting','on');
add_line(modelName, 'SatVelB/1',          'MuxVel/2',           'autorouting','on');
add_line(modelName, 'MuxVel/1',           'Scope_Velocity/1',   'autorouting','on');

add_line(modelName, 'SatDistAO/1',        'MuxDist/1',          'autorouting','on');
add_line(modelName, 'SatDistAB/1',        'MuxDist/2',          'autorouting','on');
add_line(modelName, 'MuxDist/1',          'Scope_Distance/1',   'autorouting','on');

fprintf('   ✓ All connections complete\n');

%% ── 12. LABEL SIGNALS ───────────────────────────────────────

function labelLine(mdl, srcBlk, srcPort, label)
    try
        ph = get_param([mdl '/' srcBlk], 'PortHandles');
        lh = get_param(ph.Outport(srcPort), 'Line');
        if lh > 0
            set_param(lh, 'Name', label);
        end
    catch
        % Silently skip if line doesn't exist
    end
end

labelLine(modelName, 'SatVelA',       1, 'Vel_A [m/s]');
labelLine(modelName, 'SatVelB',       1, 'Vel_B [m/s]');
labelLine(modelName, 'SatDistAO',     1, 'Dist_A→Obstacle');
labelLine(modelName, 'SatDistAB',     1, 'Dist_A→B');
labelLine(modelName, 'RelOpAO',       1, 'Alert_Trigger');
labelLine(modelName, 'AlertDelay',    1, 'V2V_Link');
labelLine(modelName, 'AndGate',       1, 'AutoBrake_Cmd');

fprintf('   ✓ Signal labels applied\n');

%% ── 13. CONFIGURE SCOPES ────────────────────────────────────

svh = get_param(P('Scope_Velocity'), 'ScopeConfiguration');
svh.OpenAtSimulationStart = true;
svh.Title    = 'Vehicle Velocities — A (ch1) vs B (ch2)';
svh.YLabel   = 'Velocity (m/s)';
svh.TimeSpan = num2str(simTime);

sdh = get_param(P('Scope_Distance'), 'ScopeConfiguration');
sdh.OpenAtSimulationStart = true;
sdh.Title    = 'Distances — A→Obstacle (ch1), A→B (ch2)';
sdh.YLabel   = 'Distance (m)';
sdh.TimeSpan = num2str(simTime);

fprintf('   ✓ Scopes configured\n');

%% ── 14. SAVE MODEL ──────────────────────────────────────────

save_system(modelName, [modelName '.slx']);
fprintf('   ✓ Model saved: %s.slx\n\n', modelName);

%% ── 15. RUN SIMULATION ──────────────────────────────────────

fprintf('▶️  Running simulation (%.1f seconds)...\n', simTime);
tic;
simOut = sim(modelName, 'StopTime', num2str(simTime));
elapsedTime = toc;
fprintf('   ✓ Simulation complete (%.2f s elapsed)\n\n', elapsedTime);

%% ── 16. ANALYTICAL VERIFICATION PLOTS ───────────────────────

fprintf('📊 Generating verification plots...\n');

% Time vector
dt = 0.001;
t  = 0:dt:simTime;

% ┌─────────────────────────────────────────────────────────┐
% │ VEHICLE A ANALYTICAL MODEL                              │
% └─────────────────────────────────────────────────────────┘
accelA = zeros(size(t));
accelA(t >= brakeStartTime) = brakeAccelA;

velA = initVelA + cumtrapz(t, accelA);
velA = max(0, velA);  % Clamp ≥ 0

posA = cumtrapz(t, velA);

% ┌─────────────────────────────────────────────────────────┐
% │ VEHICLE B ANALYTICAL MODEL                              │
% └─────────────────────────────────────────────────────────┘
tBrakeB = brakeStartTime + reactionDelay;

accelB = zeros(size(t));
accelB(t >= tBrakeB) = brakeAccelB;

velB = initVelB + cumtrapz(t, accelB);
velB = max(0, velB);

posB = cumtrapz(t, velB) - initGap_AB;

% ┌─────────────────────────────────────────────────────────┐
% │ DISTANCE CALCULATIONS                                   │
% └─────────────────────────────────────────────────────────┘
distAO = max(0, initDist_AO - posA);  % Obstacle at 50m
distAB = max(0, posA - posB);         % Gap between vehicles



% ══════════════════════════════════════════════════════════
% PLOT 1: VELOCITY vs TIME
% ══════════════════════════════════════════════════════════
figure('Name','V2V — Velocity Analysis','NumberTitle','off', ...
       'Color',[0.95 0.95 0.97],'Position',[100 500 900 450]);

subplot(2,1,1);
hold on; grid on; box on;
plot(t, velA, '-',  'Color',[0.2 0.4 0.8], 'LineWidth',2.5, 'DisplayName','Vehicle A');
plot(t, velB, '--', 'Color',[0.8 0.3 0.3], 'LineWidth',2.5, 'DisplayName','Vehicle B');
xline(brakeStartTime, ':', 'Color',[0.5 0.5 0.5], 'LineWidth',1.5, ...
      'Label','A brakes', 'LabelVerticalAlignment','bottom');
xline(tBrakeB,        ':', 'Color',[0.7 0.3 0.3], 'LineWidth',1.5, ...
      'Label',sprintf('B auto-brakes (+%.0f ms)',reactionDelay*1000), ...
      'LabelVerticalAlignment','top');
xlabel('Time (s)','FontWeight','bold');
ylabel('Velocity (m/s)','FontWeight','bold');
title('Vehicle Velocities vs Time','FontSize',12,'FontWeight','bold');
legend('Location','northeast'); 
ylim([-1 25]);  % Fixed range for 20 m/s velocity
xlim([0 simTime]);
set(gca,'FontSize',10);

% Velocity difference
subplot(2,1,2);
hold on; grid on; box on;
velDiff = velA - velB;
plot(t, velDiff, '-', 'Color',[0.3 0.6 0.4], 'LineWidth',2);
yline(0, '--k', 'LineWidth',1);
xlabel('Time (s)','FontWeight','bold');
ylabel('Vel_A - Vel_B (m/s)','FontWeight','bold');
title('Velocity Difference (Collision Risk Indicator)','FontSize',12,'FontWeight','bold');
set(gca,'FontSize',10);

% ══════════════════════════════════════════════════════════
% PLOT 2: DISTANCE vs TIME
% ══════════════════════════════════════════════════════════
figure('Name','V2V — Distance Analysis','NumberTitle','off', ...
       'Color',[0.95 0.95 0.97],'Position',[1000 500 900 450]);

subplot(2,1,1);
hold on; grid on; box on;

% Danger zone shading (distAO < 5m)
dangerMask = distAO < obstacleThresh;
if any(dangerMask)
    tDanger = t(dangerMask);
    fill([tDanger, fliplr(tDanger)], ...
         [distAO(dangerMask), zeros(1,sum(dangerMask))], ...
         [1 0.8 0.8], 'FaceAlpha',0.3, 'EdgeColor','none', ...
         'DisplayName','Danger zone');
end

plot(t, distAO, '-',  'Color',[0.2 0.4 0.8], 'LineWidth',2.5, 'DisplayName','A → Obstacle');
plot(t, distAB, '--', 'Color',[0.8 0.3 0.3], 'LineWidth',2.5, 'DisplayName','A → B gap');
yline(obstacleThresh, ':', 'Color',[0.9 0.4 0.2], 'LineWidth',1.5, ...
      'Label',sprintf('Alert threshold (%.0f m)',obstacleThresh));
xlabel('Time (s)','FontWeight','bold');
ylabel('Distance (m)','FontWeight','bold');
title('Distances vs Time','FontSize',12,'FontWeight','bold');
legend('Location','northeast');
ylim([0 initDist_AO*1.1]);
set(gca,'FontSize',10);

% Minimum distances
subplot(2,1,2);
hold on; grid on; box on;
minDistAO = min(distAO);
minDistAB = min(distAB);
[~, idxMinAO] = min(distAO);
[~, idxMinAB] = min(distAB);

plot(t, distAO, '-', 'Color',[0.2 0.4 0.8], 'LineWidth',1.5);
plot(t, distAB, '-', 'Color',[0.8 0.3 0.3], 'LineWidth',1.5);
plot(t(idxMinAO), minDistAO, 'o', 'MarkerSize',10, 'MarkerFaceColor',[0.2 0.4 0.8], ...
     'MarkerEdgeColor','k', 'LineWidth',1.5);
plot(t(idxMinAB), minDistAB, 'o', 'MarkerSize',10, 'MarkerFaceColor',[0.8 0.3 0.3], ...
     'MarkerEdgeColor','k', 'LineWidth',1.5);
text(t(idxMinAO), minDistAO+2, sprintf('Min: %.2f m @ %.2fs', minDistAO, t(idxMinAO)), ...
     'Color',[0.2 0.4 0.8], 'FontWeight','bold', 'FontSize',9);
text(t(idxMinAB), minDistAB+2, sprintf('Min: %.2f m @ %.2fs', minDistAB, t(idxMinAB)), ...
     'Color',[0.8 0.3 0.3], 'FontWeight','bold', 'FontSize',9);
xlabel('Time (s)','FontWeight','bold');
ylabel('Distance (m)','FontWeight','bold');
title('Minimum Distance Analysis','FontSize',12,'FontWeight','bold');
ylim([0 max([distAO distAB])*0.5]);
set(gca,'FontSize',10);

fprintf('   ✓ Plots generated\n\n');

%% ── 17. SAFETY REPORT ───────────────────────────────────────

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║              SAFETY ANALYSIS REPORT                  ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

fprintf('🎯 MINIMUM DISTANCES:\n');
fprintf('   • A → Obstacle:  %.2f m at t=%.2f s\n', minDistAO, t(idxMinAO));
fprintf('   • A → B gap:     %.2f m at t=%.2f s\n\n', minDistAB, t(idxMinAB));

% Collision detection
collisionAO = minDistAO < 0.5;  % < 0.5m = collision
collisionAB = minDistAB < 1.0;  % < 1.0m = collision

if collisionAO
    fprintf('❌ COLLISION DETECTED: Vehicle A hit obstacle!\n');
else
    fprintf('✅ NO COLLISION: Vehicle A stopped safely (%.2f m clearance)\n', minDistAO);
end

if collisionAB
    fprintf('❌ COLLISION DETECTED: Vehicle B hit Vehicle A!\n');
else
    fprintf('✅ NO COLLISION: Vehicles maintained safe gap (%.2f m minimum)\n', minDistAB);
end

% Alert timing
idxAlert = find(distAO < obstacleThresh, 1);
if ~isempty(idxAlert)
    tAlert = t(idxAlert);
    fprintf('\n📡 V2V COMMUNICATION:\n');
    fprintf('   • Alert triggered:   t=%.3f s (distAO=%.2f m)\n', tAlert, distAO(idxAlert));
    fprintf('   • B received alert:  t=%.3f s (delay=%.0f ms)\n', ...
            tAlert+reactionDelay, reactionDelay*1000);
    fprintf('   • B began braking:   t=%.3f s\n', tBrakeB);
end

% Stopping distances - FIXED VERSION
idxStopA = find(velA < 0.01, 1);
idxStopB = find(velB < 0.01, 1);

if ~isempty(idxStopA)
    idxBrakeStartA = find(t >= brakeStartTime, 1);
    distTraveledA = posA(idxStopA) - posA(idxBrakeStartA);
    
    fprintf('\n🛑 STOPPING PERFORMANCE:\n');
    fprintf('   • Vehicle A stopped: t=%.2f s (traveled %.2f m from brake)\n', ...
            t(idxStopA), distTraveledA);
end

if ~isempty(idxStopB)
    idxBrakeStartB = find(t >= tBrakeB, 1);
    if ~isempty(idxBrakeStartB)  % Safety check
        distTraveledB = posB(idxStopB) - posB(idxBrakeStartB);
        fprintf('   • Vehicle B stopped: t=%.2f s (traveled %.2f m from brake)\n', ...
                t(idxStopB), distTraveledB);
    end
end

fprintf('\n╔══════════════════════════════════════════════════════╗\n');
fprintf('║            ✅ SIMULATION COMPLETE ✅                 ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n');

