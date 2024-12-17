// add optional named argument

#import "@preview/physica:0.9.1": *
#import "@preview/tablex:0.0.7": tablex, hlinex, vlinex, colspanx, rowspanx
#import "@preview/codly:0.1.0": codly-init, codly, disable-codly
#show: codly-init.with()
#import "@preview/wrap-it:0.1.0": wrap-content, wrap-top-bottom
#import "@preview/showybox:2.0.1": showybox
#import "@preview/lemmify:0.1.5": *


#let blue_theorem(title,footer,body)=showybox(
  title: text(title),
  frame: (
    border-color: blue,
    title-color: blue.lighten(50%),
    body-color: white,
    footer-color: blue.lighten(95%)
  ),
  footer: text(footer)
)[ #body ]

#let dark_theorem(title,footer,body)= showybox(
  frame:(
    title-color: black.lighten(30%),
  ),
  footer-style: (
    sep-thickness: 0pt,
    align: right,
    color: black
  ),

  title: text(title),
  footer: text(footer)
)[
 #body
]

#let red_theorem(title,body)= showybox(
  frame: (
    border-color: red.darken(30%),
    title-color: red.darken(30%),
    radius: 0pt,
    thickness: 2pt,
    body-inset: 1em,
    dash: "densely-dash-dotted"
  ),
  title: text(title),
)[
  #body
]


#let purple_theorem(title,body)= showybox(
  frame: (
    border-color: purple.lighten(70%),
    title-color: purple.lighten(70%),
    radius: 0pt,
    thickness: 2pt,
    body-inset: 1em,
    dash: "densely-dash-dotted"
  ),
  title: text(title),
)[
  #body
]



/*my quote function*/

#let Quotation(name,body) = block(
  above: 2em, stroke: 0.1em + blue,
  width: 100%, inset: 14pt
)[
  #set quote(block: true, attribution: [#name])
  #set text(font: "Noto Sans", fill: black)
  #place(
    top + left,
    dy: -6pt - 14pt, // Account for inset of block
    dx: 6pt - 14pt,
    block(fill: white, inset: 2pt)[#text(blue,size: 12pt)[*Quote*]]
  )
  #align(left)[#quote[#body]]
]

#let my-thm-style(
  thm-type, name, number, body
) = grid(
  columns: (1fr, 3fr),
  column-gutter: 1em,
  stack(spacing: .5em, [#strong(thm-type) #number], emph(name)),
  body
)
#let my-styling = ( thm-styling: my-thm-style )
#let (
  definition, theorem, proof, lemma, rules
) = default-theorems("thm-group", lang: "en", ..my-styling)
#show: rules
#show thm-selector("thm-group"): box.with(inset: 0.8em)
#show thm-selector("thm-group", subgroup: "theorem"): it => box(
  it, fill: rgb("#eeffee"))


#let t= h(1.5em) //blank space for writing


#let apply-template(body, title:[ *Neural Networks相关文献阅读报告 \
  _Error bounds for approximations with deep ReLU networks_* #footnote[https://arxiv.org/abs/1610.01145]]/*an example*/,
  right_header:"数据建模与分析课程作业"/*an example*/,
left_header:"statistical learning homework"/*an example*/,
author:"Maythics"/*an example*/,
ID:"3220104133",
logo_path:"./images/CSE_logo.png",
abstract:[],
keywords:[],
column: 2
) = {
  set page(
  margin: (left:3em,right:3em),
  header: [
  #smallcaps(left_header)
  #h(1fr)
  #text(font: "BankGothic Lt BT")[#right_header] #h(3em)  /*place logo here*/
  #move(dx: 0pt, dy: -6pt,line(length: 100%))

   /*place your own logo here by delivering path*/
  #place(
  top+right,
  move(dx: 0cm, dy: 0.8cm,
  image(logo_path,width: 25pt)
   )
  )
  ],
  paper: "a4",
  numbering:"-1-",
)


  align(center, text(17pt,title))

  grid(
  columns: (1fr),
  align(center)[
    Author: #text(author) \
    ID: #text(ID) \
  ],
)

  par(justify: true)[
    #align(center)[*Abstract*]
    #align(left)[#t #abstract]

    #align(left)[*Keywords*: #keywords]
  ]

  show heading.where(
  level: 1
): it => block(width: 100%)[
  #set align(center)
  #set text(16pt, weight: "bold")
  #smallcaps(it.body)
]

  show heading.where(
  level: 2
): it => text(
  size: 13pt,
  weight: "regular",
  style: "italic",
  [#h(0.7em)]+it.body,
)

  show: rest => columns(column, rest)
  set heading(numbering: "1.1")
  set quote(block: true)
  show: rules
  show thm-selector("thm-group"): box.with(inset: 0.8em)
  show thm-selector("thm-group"): it => box(
  it, fill: luma(245))
  body
}


#let cover(subtitle:"My subTitle",
          title:"main title",
          name: "Maythics",
          teacher: "Procoffeev",
          ID:"3220104133",
          school: "ZJU",
          time:"2024",
          figure:[]
          ) = {
// author: bamdone
let accent  = rgb("#FFFFFF")
let accent1 = rgb("#AEE8F7")
let accent2 = rgb("#AEE8E6")
let accent3 = rgb("#AEE8F7")
let accent4 = rgb("#ADD8E6")
let accent5 = rgb("#AEAAE6")
let accent6 = rgb("#AEE8F7")

set page(paper: "a4",margin: 0.0in, fill: accent)

set rect(stroke: 4pt)
move(
  dx: -6cm, dy: 1.0cm,
  rotate(-45deg,
    rect(
      width: 100cm,
      height: 2cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent1,
)))

set rect(stroke: 4pt)
move(
  dx: -2cm, dy: -1.0cm,
  rotate(-45deg,
    rect(
      width: 100cm,
      height: 2cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent2,
)))

set rect(stroke: 4pt)
move(
  dx: 8cm, dy: -10cm,
  rotate(-45deg,
    rect(
      width: 100cm,
      height: 1cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent3,
)))

set rect(stroke: 4pt)
move(
  dx: 7cm, dy: -8cm,
  rotate(-45deg,
    rect(
      width: 1000cm,
      height: 2cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent4,
)))

set rect(stroke: 4pt)
move(
  dx: 4cm, dy: -4cm,
  rotate(-45deg,
    rect(
      width: 1000cm,
      height: 2cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent1,
)))

set rect(stroke: 4pt)
move(
  dx: 9cm, dy: -7cm,
  rotate(-45deg,
    rect(
      width: 1000cm,
      height: 1.5cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent6,
)))

set rect(stroke: 4pt)
move(
  dx: 16cm, dy: -13cm,
  rotate(-45deg,
    rect(
      width: 1000cm,
      height: 1cm,
      radius: 50%,
      stroke: 0pt,
      fill:accent2,
)))


place(
  top+left,
  move(dx: 2cm, dy: 0.5cm,
  image(figure,width: 100pt)
   )
  )

align(center)[
  #rect(width: 100%,
    fill: accent,
    stroke:none,
    [#align(center)[
      #text(size: 25pt,[#subtitle])
    ]
    ])
]

align(center)[
  #rect(width: 100%,
    fill: accent,
    stroke:none,
    [#align(center)[
      #text(size: 40pt,[#title])
    ]
    ])
]

v(2cm)

align(center)[
  #grid(
    columns: 2,
    rows: 4,
    gutter: 1em,

    text(size: 20pt)[姓名:],
    [#block(text(size: 20pt)[#name] )#align(bottom)[#line(length: 20em, stroke: 0.5pt)]],
    text(size: 20pt)[教师:],
    [#block(text(size: 20pt)[#teacher])#align(bottom)[#line(length: 20em, stroke: 0.5pt)]],
    text(size: 20pt)[学号:],
    [#block(text(size: 20pt)[#ID])#align(bottom)[#line(length: 20em, stroke: 0.5pt)]],
    text(size: 20pt)[院校:],
    [#block(text(size: 20pt)[#school])#align(bottom)[#line(length: 20em, stroke: 0.5pt)]],
  )
]

place(
  bottom+center,
  move(dx: 0cm, dy: -2cm,
    text(size: 17pt,[#time])
   )
  )

}


/******************************************/

#show: apply-template

#t Examples of how to use my DIY functions:

#blue_theorem("Proposition 01","footer content")[this is the body of proposition]

#dark_theorem("Gauss's Law","powered by Maythics")[#t the Flux equals to the total charges contained in an enclosed surface, i.e. $ integral.double_Sigma vectorarrow(E)dot dd(vectorarrow(S)) = integral.triple_Omega rho dd(V) $]

#red_theorem("miracle")[this is a red theorem]

#Quotation("luxin")[“聘为zju教师的学历要求起步就有‘有海外学习、科研经历并取得一定的学术成果的博士’。一个典型的晋升路径，如浙大有‘百人计划’，在六年内带领学生满足一定的教学要求，开展自己的实验室建设，并在这个平台上发表高水平的研究成果，三四年左右成为教授，之后经过院系、学部、学校的层层考核，升任长聘教授。”
]

= #lorem(3)

#lorem(300)

#proof[dbjhg]

#theorem(name: "Theorem name")[There are infinitely many primes.]<thm>
#proof[
  Suppose to the contrary that $p_1, p_2, dots, p_n$ is a finite enumeration
  of all primes. ... #highlight[_a contradiction_].]<proof>
