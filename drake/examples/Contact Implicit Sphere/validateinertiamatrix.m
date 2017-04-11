p_urdf_2D = PlanarRigidBodyManipulator('TWIP.urdf',options);
q = getZeroConfiguration(p);
v = zeros(p.getNumPositions,1);