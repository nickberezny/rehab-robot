void VirtualTrajectory(struct States * s, struct ControlParams * p);
void PeriodicReset(struct States * s);
void BasicPD(struct States * s, struct ControlParams * p);
void ComputedTorque(struct States * s, struct ControlParams * p);
void ComputedTorqueImp(struct States * s, struct ControlParams * p);