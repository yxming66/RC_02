function app = run_3pit_interactive_control()
%RUN_3PIT_INTERACTIVE_CONTROL Open the 3pit yz/pitch interactive simulator.

app = run_arm_cpp_interactive_control(model_3dof_3pit(), "3pit_hybrid");
end
