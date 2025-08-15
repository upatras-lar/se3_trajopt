class ContactScheduler:
    def __init__(self, model, dt=0.01, contact_frame_dict=None):

        self.model = model
        self.DT = dt
        self.total_nodes = 0
        self.contact_phases = []
        self.contact_sequence_fnames = []

        self.contact_frame_dict = contact_frame_dict

    def add_phase(self, phase, duration_sec):
        steps = int(duration_sec / self.DT)
        self.total_nodes += steps
        self.contact_phases.extend([phase] * steps)

        for _ in range(steps):
            frame_names = []
            for key in phase:
                frame_names.extend(self.contact_frame_dict.get(key, []))

            self.contact_sequence_fnames.append(frame_names)
