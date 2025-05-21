# state_manager.py
# -*- coding: utf-8 -*-


from state_flags import pick_status, place_status

class StateManager:
    def __init__(self):
        self.pick = pick_status
        self.place = place_status

    def mark_pick(self, huevera, obus_id, value=True):
        if huevera in self.pick and obus_id in self.pick[huevera]:
            self.pick[huevera][obus_id] = value

    def mark_place(self, huevera, obus_id, value=True):
        if huevera in self.place and obus_id in self.place[huevera]:
            self.place[huevera][obus_id] = value

    def is_picked(self, huevera, obus_id):
        return self.pick.get(huevera, {}).get(obus_id, False)

    def is_placed(self, huevera, obus_id):
        return self.place.get(huevera, {}).get(obus_id, False)

    def reset_all(self):
        for huevera in self.pick:
            for i in self.pick[huevera]:
                self.pick[huevera][i] = False
        for huevera in self.place:
            for i in self.place[huevera]:
                self.place[huevera][i] = False

    def get_pick_status(self):
        return self.pick

    def get_place_status(self):
        return self.place

