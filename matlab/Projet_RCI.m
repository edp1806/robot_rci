function Projet_RCI()
% ================================================================
% Projet Robotique : Robot SCARA à 4 degrés de liberté (ddl)
%   - Tâche 1 : Curseurs articulaires (Modèle Géométrique Direct - MGD)
%   - Tâche 2 : Curseurs cartésiens (Modèle Géométrique Inverse - MGI)
%   - Tâche 3 : Validation croisée MGD <-> MGI
%   - Tâche 4 : Trajectoire circulaire (dans la zone atteignable)
%   - Tâche 5 : Visualisation des signaux articulaires
% ================================================================

%% === Paramètres géométriques du robot ===
a = 1.85;   % Longueur principale du bras (en mètres)
b = 0.35;   % Décalage de l'outil par rapport à l'effecteur (en mètres)

%% --- Modèle Géométrique Direct (MGD) ---
% Entrée : q = [q1, q2, q3, q4] (angles et translation en radians/mètres)
% Sortie : p3 = [X, Y, Z] (position cartésienne de l'effecteur)
    function p3 = MGD(q, a, b)
        q1 = q(1); % Angle de rotation du premier joint (rad)
        q2 = q(2); % Angle de rotation du deuxième joint (rad)
        q4 = q(4); % Translation du quatrième joint (m)
        % Calcul des coordonnées cartésiennes de l'effecteur
        xp3 = (a + (q4 + b) * sin(q2)) * sin(q1);
        yp3 = -(a + (q4 + b) * sin(q2)) * cos(q1);
        zp3 = (q4 + b) * cos(q2) + 0.2; % +0.2 m pour la hauteur de la base
        p3 = [xp3, yp3, zp3];
    end

%% --- Modèle Géométrique Inverse (MGI) ---
% Entrée : p3 = [X, Y, Z] (position cartésienne)
% Sortie : qd = [q1, q2, q3, q4] (angles et translation)
    function qd = MGI(p3, a, b)
        X = p3(1);
        Y = p3(2);
        Z = p3(3);
        % Calcul de q1 (angle de rotation autour de Z)
        q1 = atan2(X, -Y);
        % Calcul de q2 (angle de rotation autour de X)
        q2 = atan2(sqrt(X^2 + Y^2) - a, Z - 0.2);
        % q3 est arbitrairement fixé à 0 (pas de rotation autour de Z pour l'outil)
        q3 = 0;
        % Calcul de q4 (translation le long de Z)
        q4 = sqrt((sqrt(X^2 + Y^2) - a)^2 + (Z - 0.2)^2) - b;
        qd = [q1, q2, q3, q4];
    end

%% === Construction du modèle du robot (rigidBodyTree) ===
robot = rigidBodyTree;

% --- Joint 1 : Rotation autour de Z (q1) ---
B1 = rigidBody('Body1');
J1 = rigidBodyJoint('Joint1','revolute');
J1.JointAxis = [0 0 1]; % Axe de rotation Z
setFixedTransform(J1, trvec2tform([0 0 0.2])); % Positionnement du joint
B1.Joint = J1;
addBody(robot, B1, 'base');

% --- Joint 2 : Rotation autour de X (q2) ---
B2 = rigidBody('Body2');
J2 = rigidBodyJoint('Joint2','revolute');
J2.JointAxis = [1 0 0]; % Axe de rotation X
setFixedTransform(J2, trvec2tform([0 -a 0])); % Décalage de 'a' sur l'axe Y
B2.Joint = J2;
addBody(robot, B2, 'Body1');

% --- Joint 3 : Rotation autour de Z (q3) ---
B3 = rigidBody('Body3');
J3 = rigidBodyJoint('Joint3','revolute');
J3.JointAxis = [0 0 1]; % Axe de rotation Z
setFixedTransform(J3, trvec2tform([0 0 b])); % Décalage de 'b' sur l'axe Z
B3.Joint = J3;
addBody(robot, B3, 'Body2');

% --- Joint 4 : Translation le long de Z (q4) ---
B4 = rigidBody('Body4');
J4 = rigidBodyJoint('Joint4','prismatic');
J4.JointAxis = [0 0 1]; % Axe de translation Z
setFixedTransform(J4, trvec2tform([0 0 0]));
B4.Joint = J4;
addBody(robot, B4, 'Body3');

%% === Position initiale du robot ===
config = homeConfiguration(robot);
config(1).JointPosition = deg2rad(30); % q1 = 30°
config(2).JointPosition = deg2rad(40); % q2 = 40°
config(3).JointPosition = 0;          % q3 = 0°
config(4).JointPosition = 0;          % q4 = 0 m

%% === Fenêtre principale pour la visualisation 3D ===
fig = figure('Name','Projet RCI');
ax = axes('Parent',fig);
hold(ax,'on');
grid on;
axis equal;
view(3);

%% === Nuage bleu : Espace de travail du robot ===
% Discrétisation des angles et translations pour calculer l'espace atteignable
q1_values = linspace(deg2rad(-110), deg2rad(110), 40);
q2_values = linspace(deg2rad(0), deg2rad(80), 30);
q4_values = linspace(0, 0.15, 20);
workspace_points = zeros(length(q1_values) * length(q2_values) * length(q4_values), 3);
idx = 1;
for q1 = q1_values
    for q2 = q2_values
        for q4 = q4_values
            p3 = MGD([q1, q2, 0, q4], a, b);
            workspace_points(idx, :) = p3;
            idx = idx + 1;
        end
    end
end
workspace_points(idx:end, :) = [];
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 5, '.', 'b');

% Affichage du robot et de l'effecteur (point rouge)
show(robot, config, 'Parent', ax, 'PreservePlot', true, 'Frames', 'on');
p3 = MGD([config(1).JointPosition, config(2).JointPosition, 0, config(4).JointPosition], a, b);
hEff = plot3(ax, p3(1), p3(2), p3(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

%% === Interface utilisateur (UI) ===
f = uifigure('Name','Contrôle du robot','Position',[100 100 950 550]);

% --- Curseurs articulaires (Tâche 1) ---
panelQ = uipanel(f,'Title','Tâche 1 : Curseurs articulaires (MGD)','Position',[20 150 400 350]);
sld1 = uislider(panelQ,'Position',[80 250 250 3],'Limits',[-110 110],'Value',30);
sld2 = uislider(panelQ,'Position',[80 200 250 3],'Limits',[0 80],'Value',40);
sld3 = uislider(panelQ,'Position',[80 150 250 3],'Limits',[0 360],'Value',0);
sld4 = uislider(panelQ,'Position',[80 100 250 3],'Limits',[0 15],'Value',10);

% --- Curseurs cartésiens (Tâche 2) ---
panelC = uipanel(f,'Title','Tâche 2 : Curseurs cartésiens (MGI)','Position',[450 150 400 350]);
sldX = uislider(panelC,'Position',[80 250 250 3],'Limits',[-2 2],'Value',p3(1));
sldY = uislider(panelC,'Position',[80 200 250 3],'Limits',[-2 2],'Value',p3(2));
sldZ = uislider(panelC,'Position',[80 150 250 3],'Limits',[0 1],'Value',p3(3));

% --- Boutons pour les tâches 3, 4 et 5 ---
uibutton(panelC,'push','Text','Validation MGD <-> MGI (Tâche 3)', ...
    'Position',[80 80 250 30],'ButtonPushedFcn',@validateCross);
uibutton(f,'push','Text','Tracer un cercle (Tâche 4)', ...
    'Position',[350 70 250 40],'ButtonPushedFcn',@trajCercle);
uibutton(f,'push','Text','Visualiser signaux articulaires (Tâche 5)', ...
    'Position',[350 20 250 40],'ButtonPushedFcn',@trajSignaux);

%% === Fonctions locales ===
    % --- Tâche 1 : Mise à jour par curseurs articulaires ---
    function updateFromQ(~,~)
        q1 = deg2rad(sld1.Value);
        q2 = deg2rad(sld2.Value);
        q3 = deg2rad(sld3.Value);
        q4 = sld4.Value/100; % Conversion de cm à m
        % Mise à jour de la configuration du robot
        config = homeConfiguration(robot);
        config(1).JointPosition = q1;
        config(2).JointPosition = q2;
        config(3).JointPosition = q3;
        config(4).JointPosition = q4;
        % Affichage du robot
        show(robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'on');
        % Calcul de la position de l'effecteur
        p3 = MGD([q1, q2, q3, q4], a, b);
        % Mise à jour de l'affichage de l'effecteur
        set(hEff, 'XData', p3(1), 'YData', p3(2), 'ZData', p3(3));
        % Synchronisation des curseurs cartésiens
        sldX.Value = min(max(p3(1), sldX.Limits(1)), sldX.Limits(2));
        sldY.Value = min(max(p3(2), sldY.Limits(1)), sldY.Limits(2));
        sldZ.Value = min(max(p3(3), sldZ.Limits(1)), sldZ.Limits(2));
    end

    % --- Tâche 2 : Mise à jour par curseurs cartésiens ---
    function updateFromXYZ(~,~)
        x = sldX.Value;
        y = sldY.Value;
        z = sldZ.Value;
        q = MGI([x, y, z], a, b);
        config = homeConfiguration(robot);
        config(1).JointPosition = q(1);
        config(2).JointPosition = q(2);
        config(3).JointPosition = q(3);
        config(4).JointPosition = q(4);
        show(robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'on');
        set(hEff, 'XData', x, 'YData', y, 'ZData', z);
        % Synchronisation des curseurs articulaires
        sld1.Value = min(max(rad2deg(q(1)), sld1.Limits(1)), sld1.Limits(2));
        sld2.Value = min(max(rad2deg(q(2)), sld2.Limits(1)), sld2.Limits(2));
        sld3.Value = min(max(rad2deg(q(3)), sld3.Limits(1)), sld3.Limits(2));
        sld4.Value = min(max(100*q(4), sld4.Limits(1)), sld4.Limits(2));
    end

    % --- Tâche 3 : Validation croisée MGD <-> MGI ---
    function validateCross(~,~)
        q1 = deg2rad(30);
        q2 = deg2rad(40);
        q3 = 0;
        q4 = 0.12;
        p3 = MGD([q1, q2, q3, q4], a, b);
        qd = MGI(p3, a, b);
        p3_check = MGD(qd, a, b);
        % Calcul des erreurs
        errQ = [rad2deg(q1-qd(1)), rad2deg(q2-qd(2)), q3-qd(3), (q4-qd(4))*100];
        errXYZ = [p3(1)-p3_check(1), p3(2)-p3_check(2), p3(3)-p3_check(3)];
        % Affichage des résultats
        uialert(f, sprintf(['=== Validation ===\nErreur q : [%.2f°, %.2f°, %.2f rad, %.2f cm]\nErreur pos : [%.2e, %.2e, %.2e] m'], ...
            errQ(1), errQ(2), errQ(3), errQ(4), errXYZ(1), errXYZ(2), errXYZ(3)), ...
            'Validation MGD <-> MGI');
    end

    % --- Tâche 4 : Tracer un cercle ---
    function trajCercle(~, ~)
        % Paramètres du cercle (ajustés pour être dans l'espace atteignable)
        xc = -1.70;
        yc = -1.10;
        zc = 0.55;
        R = 0.20;
        theta = linspace(0, 2*pi, 60);
        X = xc + R*cos(theta);
        Y = yc + R*sin(theta);
        Z = zc*ones(size(theta));
        % Tracé du cercle
        plot3(ax, X, Y, Z, 'g--', 'LineWidth', 1.5);
        for i = 1:length(X)
            q = MGI([X(i), Y(i), Z(i)], a, b);
            config = homeConfiguration(robot);
            config(1).JointPosition = q(1);
            config(2).JointPosition = q(2);
            config(3).JointPosition = q(3);
            config(4).JointPosition = q(4);
            show(robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'on');
            set(hEff, 'XData', X(i), 'YData', Y(i), 'ZData', Z(i));
            plot3(ax, X(i), Y(i), Z(i), 'r.', 'MarkerSize', 8);
            pause(0.05);
        end
    end

    % --- Tâche 5 : Visualisation des signaux articulaires ---
    function trajSignaux(~, ~)
        % Paramètres du cercle
        xc = -1.70;
        yc = -1.10;
        zc = 0.55;
        R = 0.20;
        theta = linspace(0, 2*pi, 100);
        % Coordonnées cartésiennes du cercle
        X = xc + R*cos(theta);
        Y = yc + R*sin(theta);
        Z = zc*ones(size(theta));
        % Initialisation des vecteurs pour les signaux articulaires
        Q1 = []; % q1 en radians
        Q2 = []; % q2 en radians
        Q3 = []; % q3 en radians
        Q4 = []; % q4 en mètres
        % Calcul des coordonnées articulaires pour chaque point du cercle
        for i = 1:length(X)
            q = MGI([X(i), Y(i), Z(i)], a, b);
            Q1(end+1) = q(1);
            Q2(end+1) = q(2);
            Q3(end+1) = q(3);
            Q4(end+1) = q(4);
        end
        % Temps ou itérations (pour l'axe des abscisses)
        temps = linspace(0, 2*pi, length(X));
        % Affichage des signaux articulaires en fonction du temps
        figure;
        subplot(4,1,1);
        plot(temps, rad2deg(Q1), 'LineWidth', 1.5, 'Color', 'b');
        ylabel('q1 (°)');
        title('Évolution des angles articulaires');
        grid on;
        subplot(4,1,2);
        plot(temps, rad2deg(Q2), 'LineWidth', 1.5, 'Color', 'r');
        ylabel('q2 (°)');
        grid on;
        subplot(4,1,3);
        plot(temps, rad2deg(Q3), 'LineWidth', 1.5, 'Color', 'g');
        ylabel('q3 (°)');
        grid on;
        subplot(4,1,4);
        plot(temps, Q4*100, 'LineWidth', 1.5, 'Color', 'm');
        ylabel('q4 (cm)');
        xlabel('Temps (s)');
        grid on;
        sgtitle('Signaux articulaires en fonction du temps pour le suivi du cercle');
    end

%% === Liaison des sliders avec les callbacks ===
sld1.ValueChangedFcn = @updateFromQ;
sld2.ValueChangedFcn = @updateFromQ;
sld3.ValueChangedFcn = @updateFromQ;
sld4.ValueChangedFcn = @updateFromQ;
sldX.ValueChangedFcn = @updateFromXYZ;
sldY.ValueChangedFcn = @updateFromXYZ;
sldZ.ValueChangedFcn = @updateFromXYZ;

end
