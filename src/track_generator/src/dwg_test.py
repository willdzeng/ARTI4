import svgwrite
from svgwrite import cm, mm

def basic_shapes(name):
    dwg = svgwrite.Drawing(filename=name, debug=True)
    # hlines = dwg.add(dwg.g(id='hlines', stroke='green'))
    # for y in range(20):
    #     hlines.add(dwg.line(start=(2*cm, (2+y)*cm), end=(18*cm, (2+y)*cm)))
    # vlines = dwg.add(dwg.g(id='vline', stroke='blue'))
    # for x in range(17):
    #     vlines.add(dwg.line(start=((2+x)*cm, 2*cm), end=((2+x)*cm, 21*cm)))
    # shapes = dwg.add(dwg.g(id='shapes', fill='red'))
    #
    # # set presentation attributes at object creation as SVG-Attributes
    # shapes.add(dwg.circle(center=(15*cm, 8*cm), r='2.5cm', stroke='blue',
    #                       stroke_width=3))
    c1 = dwg.circle(center=(0,0),r='1000mm')
    print c1.Polyline.points

    # # override the 'fill' attribute of the parent group 'shapes'
    # shapes.add(dwg.rect(insert=(5*cm, 5*cm), size=(45*mm, 45*mm),
    #                     fill='blue', stroke='red', stroke_width=3))
    #
    # # or set presentation attributes by helper functions of the Presentation-Mixin
    # ellipse = shapes.add(dwg.ellipse(center=(10*cm, 15*cm), r=('5cm', '10mm')))
    # ellipse.fill('green', opacity=0.5).stroke('black', width=5).dasharray([20, 20])
    # dwg.save()

if __name__ == '__main__':
    basic_shapes('basic_shapes.svg')