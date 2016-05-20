import xml.etree.ElementTree as ET
import copy
import numpy

# every thing with an _ is a xml object
def main():
    global tile_interval
    tile_interval = 0.27
    tree_ = ET.parse('model_template.sdf')
    joint_tmp_ = ET.parse('joint_template.sdf').getroot()
    link_tmp_ = ET.parse('link_template.sdf').getroot()
    root_ = tree_.getroot()
    model_ = root_[0]
    tile_number = 20
    links = []
    joints = []

    for i in range(tile_number):
        link = Link(link_tmp_, i)
        link.change_pose([tile_interval*i,0,0,0,0,0])
        links.append(link)
    for i in range(0,len(links)-1):
        joint = Joint(joint_tmp_,links[i],links[i+1])
        joints.append(joint)

    for link in links:
        print link
        model_.append(link.link_)
    for joint in joints:
        print joint
        model_.append(joint.joint_)

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
    global tile_interval
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
        self.pose = [tile_interval/2 ,0,0,0,0,0] #(child_link.pose + parent_link.pose)/2
        self.pose_.text = ''.join('%1.4f '%x for x in self.pose)

    def __str__(self):
        return 'Joint:%s Pose: %s '%(self.name,self.pose_.text)

class Link:
    def __init__(self, link, id):
        self.id = id
        self.pose = numpy.float32([0,0,0,0,0,0])
        self.link_ = copy.deepcopy(link)
        self.pose_ = self.link_.find('pose')
        self.pose_.text = ''.join('%1.4f ' % x for x in self.pose)
        self.name = 'link_%d' % id
        self.link_.attrib['name'] = self.name

    def __str__(self):
        return 'Link:%s Pose: %s '%(self.name,self.pose_.text)

    def change_pose(self,pose):
        if not isinstance(pose,numpy.ndarray):
            pose = numpy.float32(pose)
        self.pose = pose
        self.pose_.text = ''.join('%1.4f '%x for x in self.pose)

if __name__ == '__main__':
    main()
