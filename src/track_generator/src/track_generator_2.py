import xml.etree.ElementTree as ET
import copy
import numpy


# every thing with an _ is a xml object
def main():
    # overall_scale = 0.1
    # link_scale = [0.1,0.05,0.1]
    overall_scale = 1
    global joint_interval
    joint_interval = 0.024178 * overall_scale
    joint_limit = 0.35
    tree_ = ET.parse('model_template.sdf')
    joint_tmp_ = ET.parse('joint_template.sdf').getroot()
    link_tmp_ = ET.parse('link_template_2.sdf').getroot()
    root_ = tree_.getroot()
    model_ = root_[0]
    tile_number = 33
    links = []
    joints = []

    PI = numpy.pi
    theta = 2 * PI / tile_number
    radius = joint_interval / 2 / numpy.tan(theta / 2)

    # generate links
    for i in range(tile_number):
        link = Link(link_tmp_, i)
        link.change_pose([radius * numpy.cos(theta * i), radius * numpy.sin(theta * i), 0, -PI/2, 0, PI/2 + theta * i])
        # link.change_scale(link_scale)
        links.append(link)
        print link.pose

    # generate joints
    for i in range(0, len(links) - 1):
        joint = Joint(joint_tmp_, links[i], links[i + 1])
        joint.change_limits(joint_limit)
        joints.append(joint)

    # connect the last two joints
    joint = Joint(joint_tmp_,links[-1],links[0])
    joints.append(joint)

    # add the links and joint into model
    for link in links:
        print link
        model_.append(link.link_)
    for joint in joints:
        print joint
        model_.append(joint.joint_)

    # write files
    tree_.write('../model/track_thread/model.sdf')


def get_temp_link(model):
    tmp_link = None
    for child in model:
        if child.tag == 'link':
            tmp_link = copy.deepcopy(child)
            break
    return tmp_link


def get_all_link(model):
    links = []
    for child in model:
        if child.tag == 'link':
            links.append(child)
    return links


class Joint:
    global joint_interval

    def __init__(self, joint_tmp, child_link, parent_link):
        self.child_link = child_link
        self.parent_link = parent_link
        self.joint_ = copy.deepcopy(joint_tmp)
        self.name = 'joint_%d_%d' % (child_link.id, parent_link.id)
        # update child and parent name
        self.child_ = self.joint_.find('child')
        self.parent_ = self.joint_.find('parent')
        self.child_.text = self.child_link.name
        self.parent_.text = self.parent_link.name
        # update joint name
        self.joint_.attrib['name'] = self.name
        # calculate the pose of the joint by interpolate the child pose and parent pose
        self.pose_ = self.joint_.find('pose')
        self.pose = [joint_interval / 2, 0, 0, 0, 0, 0]  # (child_link.pose + parent_link.pose)/2
        self.pose_.text = ''.join('%1.4f ' % x for x in self.pose)

    def change_limits(self, limit):
        for x in self.joint_.iter(tag='lower'):
            x.text = '%1.3f' % -limit
        for x in self.joint_.iter(tag='upper'):
            x.text = '%1.3f' % limit

    def __str__(self):
        return 'Joint:%s Pose: %s ' % (self.name, self.pose_.text)


class Link:
    def __init__(self, link, id):
        self.id = id
        self.pose = numpy.float32([0, 0, 0, 0, 0, 0])
        self.link_ = copy.deepcopy(link)
        self.pose_ = self.link_.find('pose')
        self.pose_.text = ''.join('%1.4f ' % x for x in self.pose)
        self.name = 'link_%d' % id
        self.link_.attrib['name'] = self.name

    def __str__(self):
        return 'Link:%s Pose: %s ' % (self.name, self.pose_.text)

    def change_pose(self, pose):
        if not isinstance(pose, numpy.ndarray):
            pose = numpy.float32(pose)
        self.pose = pose
        self.pose_.text = ''.join('%1.4f ' % x for x in self.pose)

    def change_scale(self, scale):
        self.scale = scale
        if not isinstance(scale, list):
            self.scale = [scale for x in range(3)]
        for x in self.link_.iter(tag='scale'):
            x.text = ''.join('%1.3f ' % num for num in self.scale)


if __name__ == '__main__':
    main()
