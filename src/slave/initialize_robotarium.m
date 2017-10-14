function robotarium = initialize_robotarium(N,L)

rb = RobotariumBuilder();
r = rb.set_number_of_agents(N).set_save_data(false).set_show_figure(true).build();

linearVelocityGain = 1;
angularVelocityGain = pi/2;

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.06);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ...
    'AngularVelocityLimit', angularVelocityGain);

idcs_all = find(L==-1);
idcs = [];
for i = 1 : length(idcs_all)
    [I,J] = ind2sub(size(L),idcs_all(i));
    if J > I
        idcs(end+1) = idcs_all(i);
    end
end

robotarium.r = r;
robotarium.si_barrier_cert = si_barrier_cert;
robotarium.si_to_uni_dyn = si_to_uni_dyn;
robotarium.L = L;
robotarium.idcs = idcs;

end