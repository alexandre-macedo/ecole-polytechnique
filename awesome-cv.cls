%% Start of file `awesome-cv.cls'.
% Awesome CV Class File
%
% This class has been downloaded from:
% https://github.com/posquit0/Awesome-CV
%
% Author:
% Claud D. Park <posquit0.bj@gmail.com>
% http://www.posquit0.com
%
% Notes:
% 1) This class file defines the structure and layout of the template file (cv.tex, resume.tex).
% 2) It has been written in such a way that under most circumstances you
% should not need to edit it.
%
% Class license:
% LPPL v1.3c (http://www.latex-project.org/lppl)
%


%-------------------------------------------------------------------------------
%                Identification
%-------------------------------------------------------------------------------
\ProvidesClass{awesome-cv}[2015/12/14 v1.3 Awesome Curriculum Vitae Class]
\NeedsTeXFormat{LaTeX2e}


%-------------------------------------------------------------------------------
%                Class options
%
% (need to be done before the external package loading, for example because
% we need \paperwidth, \paperheight and \@ptsize to be defined before loading
% geometry and fancyhdr)
%-------------------------------------------------------------------------------
% Options for draft or final
\DeclareOption{draft}{\setlength\overfullrule{5pt}}
\DeclareOption{final}{\setlength\overfullrule{0pt}}
% Inherit options of article
\DeclareOption*{
  \PassOptionsToClass{\CurrentOption}{article}
}
\ProcessOptions\relax
\LoadClass{article}


%-------------------------------------------------------------------------------
%                3rd party packages
%-------------------------------------------------------------------------------
% Needed to make fixed length table
\RequirePackage{array}
% Needed to handle list environment
\RequirePackage{enumitem}
% Needed to handle text alignment
\RequirePackage{ragged2e}
% Needed to configure page layout
\RequirePackage{geometry}
% Needed to make header & footer effeciently
\RequirePackage{fancyhdr}
% Needed to manage colors
\RequirePackage{xcolor}
% Needed to use \ifxetex-\else-\fi statement
\RequirePackage{ifxetex}
% Needed to use \if-\then-\else statement
\RequirePackage{xifthen}
% Needed to use a toolbox of programming tools
\RequirePackage{etoolbox}
% Needed to change line spacing in specific environment
\RequirePackage{setspace}
% Needed to manage fonts
\ifxetex
  \RequirePackage[quiet]{fontspec}
  % To support LaTeX quoting style
  \defaultfontfeatures{Ligatures=TeX}
\else
  \RequirePackage[T1]{fontenc}
  % Replace by the encoding you are using
  \RequirePackage[utf8]{inputenc}
\fi
% Needed to manage math fonts
\RequirePackage[math-style=TeX]{unicode-math}
% Needed to use icons from font-awesome
% (https://github.com/posquit0/latex-fontawesome)
\RequirePackage{fontawesome}
% Needed to deal a paragraphs
\RequirePackage{parskip}
% Needed to deal hyperlink
\RequirePackage{hyperref}
\hypersetup{
  pdftitle={},
  pdfauthor={},
  pdfsubject={},
  pdfkeywords={},
  colorlinks=false,
  allbordercolors=white
}


%-------------------------------------------------------------------------------
%                Configuration for directory locations
%-------------------------------------------------------------------------------
% Configure a directory location for fonts(default: 'fonts/')
\newcommand*{\fontdir}[1][fonts/]{\def\@fontdir{#1}}
\fontdir


%-------------------------------------------------------------------------------
%                Configuration for layout
%-------------------------------------------------------------------------------
%% Page Layout
% Configure page margins with geometry
\geometry{left=2.0cm, top=1.5cm, right=2.0cm, bottom=2.0cm, footskip=.5cm}

%% Header & Footer
% Set offset to each header and offset
\fancyhfoffset{0em}
% Remove head rule
\renewcommand{\headrulewidth}{0pt}
% Clear all header & footer fields
\fancyhf{}
% Enable if you want to make header or footer using fancyhdr
\pagestyle{fancy}


%-------------------------------------------------------------------------------
%                Configuration for colors
%-------------------------------------------------------------------------------
% Gray-scale colors
\definecolor{white}{HTML}{FFFFFF}
\definecolor{black}{HTML}{000000}
\definecolor{darkgray}{HTML}{333333}
\definecolor{gray}{HTML}{5D5D5D}
\definecolor{lightgray}{HTML}{999999}
% Basic colors
\definecolor{green}{HTML}{C2E15F}
\definecolor{orange}{HTML}{FDA333}
\definecolor{purple}{HTML}{D3A4F9}
\definecolor{red}{HTML}{FB4485}
\definecolor{blue}{HTML}{6CE0F1}
% Text colors
\definecolor{darktext}{HTML}{414141}
\colorlet{text}{darkgray}
\colorlet{graytext}{gray}
\colorlet{lighttext}{lightgray}

%-------------------------------------------------------------------------------
%                Configuration for fonts
%-------------------------------------------------------------------------------
% Set font for header (default is Roboto)
\newfontfamily\headerfont[
  Path=\@fontdir,
  UprightFont=*-Regular,
  ItalicFont=*-Italic,
  BoldFont=*-Bold,
  BoldItalicFont=*-BoldItalic,
]{Roboto}

\newfontfamily\headerfontlight[
  Path=\@fontdir,
  UprightFont=*-Thin,
  ItalicFont=*-ThinItalic,
  BoldFont=*-Medium,
  BoldItalicFont=*-MediumItalic,
]{Roboto}

% Set font for footer (default is Source Sans Pro)
\newfontfamily\footerfont[
  Path=\@fontdir,
  UprightFont=*-Regular,
  ItalicFont=*-It,
  BoldFont=*-Bold
]{SourceSansPro}

% Set font for body (default is Source Sans Pro)
\newfontfamily\bodyfont[
  Path=\@fontdir,
  UprightFont=*-Regular,
  ItalicFont=*-It,
  BoldFont=*-Bold,
  BoldItalicFont=*-BoldIt
]{SourceSansPro}

\newfontfamily\bodyfontlight[
  Path=\@fontdir,
  UprightFont=*-Light,
  ItalicFont=*-LightIt,
  BoldFont=*-Semibold,
  BoldItalicFont=*-SemiboldIt
]{SourceSansPro}


%-------------------------------------------------------------------------------
%                Configuration for styles
%-------------------------------------------------------------------------------
% Configure styles for each CV elements
% For fundamental structures
\newcommand*{\headerfirstnamestyle}[1]{{\fontsize{28pt}{1em}\headerfontlight\color{graytext} #1}}
\newcommand*{\headerlastnamestyle}[1]{{\fontsize{28pt}{1em}\headerfont\bfseries\color{text} #1}}
\newcommand*{\headeraddressstyle}[1]{{\fontsize{8pt}{1em}\headerfont\itshape\color{lighttext} #1}}
\newcommand*{\headersocialstyle}[1]{{\fontsize{6.8pt}{1em}\headerfont\color{text} #1}}
\newcommand*{\headerstatementstyle}[1]{{\fontsize{9pt}{1em}\bodyfont\itshape\color{darktext} #1}}
\newcommand*{\footerstyle}[1]{{\fontsize{8pt}{1em}\footerfont\scshape\color{lighttext} #1}}
\newcommand*{\sectionstyle}[1]{{\fontsize{16pt}{1em}\bodyfont\bfseries\color{text} #1}}

% For elements of entry
\newcommand*{\entrytitlestyle}[1]{{\fontsize{10pt}{1em}\bodyfont\bfseries\color{darktext} #1}}
\newcommand*{\entrypositionstyle}[1]{{\fontsize{8pt}{1em}\bodyfont\scshape\color{graytext} #1}}
\newcommand*{\entrydatestyle}[1]{{\fontsize{8pt}{1em}\bodyfontlight\slshape\color{graytext} #1}}
\newcommand*{\entrylocationstyle}[1]{{\fontsize{9pt}{1em}\bodyfontlight\slshape\color{graytext} #1}}
\newcommand*{\descriptionstyle}[1]{{\fontsize{9pt}{1em}\bodyfontlight\upshape\color{text} #1}}

% For elements of skill
\newcommand*{\skilltypestyle}[1]{{\fontsize{10pt}{1em}\bodyfont\bfseries\color{darktext} #1}}
\newcommand*{\skillsetstyle}[1]{{\fontsize{9pt}{1em}\bodyfontlight\color{text} #1}}

%-------------------------------------------------------------------------------
%                Commands for personal information
%-------------------------------------------------------------------------------
% Define writer's name
% Usage: \name{<firstname>}{<lastname>}
% Usage: \firstname{<firstname>}
% Usage: \lastname{<lastname>}
% Usage: \familyname{<familyname>}
\newcommand*{\name}[2]{\def\@firstname{#1}\def\@lastname{#2}}
\newcommand*{\firstname}[1]{\def\@firstname{#1}}
\newcommand*{\lastname}[1]{\def\@lastname{#1}}
\newcommand*{\familyname}[1]{\def\@lastname{#1}}
\def\@familyname{\@lastname}

% Define writer's address
% Usage: \address{<address>}
\newcommand*{\address}[1]{\def\@address{#1}}

% Defines writer's mobile (optional)
% Usage: \mobile{<mobile number>}
\newcommand*{\mobile}[1]{\def\@mobile{#1}}

% Defines writer's email (optional)
% Usage: {<email adress>}
\newcommand*{\email}[1]{\def\@email{#1}}

% Defines writer's linked-in (optional)
% Usage: \linkedin{<linked-in-nick>}
\newcommand*{\linkedin}[1]{\def\@linkedin{#1}}

% Defines writer's nationality (optional)
% Usage: \nationality{<nationality>}
\newcommand*{\nationality}[1]{\def\@nationality{#1}}

% Defines writer's birth day (optional)
% Usage: \brithday{<birthday>}
\newcommand*{\birthday}[3]{\def\@birthday{#1} \def\@birthmonth{#2} \def\@birthyear{#3}}

% Defines writer's statement (optional)
% Usage: \statement{<statement>}
\newcommand*{\statement}[1]{\def\@statement{#1}}

%-------------------------------------------------------------------------------
%                Commands for extra
%-------------------------------------------------------------------------------
% Define separator for social informations in header
% Usage: \headersocialsep{<separator>}
% Default: \quad\textbar\quad
\newcommand*{\headersocialsep}[1][\quad\textbar\quad]{\def\@headersocialsep{#1}}
\headersocialsep

\newcommand*{\calculateage}[3]{
  \newcounter{myage}
  \setcounter{myage}{\the\year}
  \addtocounter{myage}{-#3}
  \ifthenelse{\the\month<#2}{\addtocounter{myage}{-1}}{}
  \ifthenelse{\the\month=#2}{
    \ifthenelse{\the\day < #1}{\addtocounter{myage}{-1}}{}
  }{}
  \hspace{-3mm}\arabic{myage} \ years
}


%-------------------------------------------------------------------------------
%                Commands for utilities
%-------------------------------------------------------------------------------
% Use to align an element of tabular table
\newcolumntype{L}[1]{>{\raggedright\let\newline\\\arraybackslash\hspace{0pt}}m{#1}}
\newcolumntype{C}[1]{>{\centering\let\newline\\\arraybackslash\hspace{0pt}}m{#1}}
\newcolumntype{R}[1]{>{\raggedleft\let\newline\\\arraybackslash\hspace{0pt}}m{#1}}

% Use to draw horizontal line with specific tickness
\def\vhrulefill#1{\leavevmode\leaders\hrule\@height#1\hfill \kern\z@}

% Use to execute conditional statements by checking empty string
\newcommand*{\ifempty}[3]{\ifthenelse{\isempty{#1}}{#2}{#3}}


%-------------------------------------------------------------------------------
%                Commands for elements of CV structure
%-------------------------------------------------------------------------------
% Define a header for CV
% Usage: \makecvheader
\newcommand*{\makecvheader}{
  \begin{center}
    \headerfirstnamestyle{
      \@firstname
    } \headerlastnamestyle{
      \@lastname
    }
    \\
    \vspace{0.8mm}
    \ifthenelse{\isundefined{\@address}}{}{\headeraddressstyle{\@address\\}}
    \vspace{-0.5mm}
    \headersocialstyle{
      \newbool{isstart}
      \setbool{isstart}{true}
      \ifthenelse{\isundefined{\@mobile}}
        {}
        {
          \faMobile\ \@mobile
          \setbool{isstart}{false}
        }
      \ifthenelse{\isundefined{\@email}}
        {}
        {
          \ifbool{isstart}
            {
              \setbool{istart}{false}
            }
            {\@headersocialsep}
          \href{mailto:\@email}{\faEnvelope\ \@email}
        }
      \ifthenelse{\isundefined{\@linkedin}}
        {}
        {
          \ifbool{isstart}
            {
              \setbool{istart}{false}
            }
            {\@headersocialsep}
          \href{https://www.linkedin.com/in/\@linkedin}{\faLinkedinSquare\ \@linkedin}
        }
      \ifthenelse{\isundefined{\@twitter}}
        {}
        {
          \ifbool{isstart}
            {
              \setbool{istart}{false}
            }
            {\@headersocialsep}
          \href{https://twitter.com/\@twitter}{\faTwitter\ \@twitter}
        }
        \ifthenelse{\isundefined{\@nationality}}
          {}
          {
            \ifbool{isstart}
              {
                \setbool{istart}{false}
              }
              {\@headersocialsep}
            \faMap\ \@nationality
          }
          \ifthenelse{\isundefined{\@birthday} \AND \isundefined{\@birthmonth} \AND \isundefined{\@birthyear}}
            {}
            {
              \ifbool{isstart}
                {
                  \setbool{istart}{false}
                }
                {\@headersocialsep}
              \faChild\ \calculateage{\@birthday}{\@birthmonth}{\@birthyear}
            }
    } \\
    \ifthenelse{\isundefined{\@statement}}
      {}
      {\vspace{2.0mm}\headerstatementstyle{\@statement\\}\vspace{2.0mm}}
  \end{center}
}

% Define a footer for CV
% Usage: \makecvfooter{<left>}{<center>}{<right>}
\newcommand*{\makecvfooter}[3]{
  \fancyfoot{}
  \fancyfoot[L]{
    \footerstyle{#1}
  }
  \fancyfoot[C]{
    \footerstyle{#2}
  }
  \fancyfoot[R]{
    \footerstyle{#3}
  }
}

% Define a section for CV
% Usage: \cvsection{<section-title>}
\newcommand{\cvsection}[1]{
  \par\addvspace{1.ex}
  \phantomsection{}
  \sectionstyle{#1}
  \color{gray}\vhrulefill{0.9pt}
  \par\nobreak\addvspace{1ex}
}

% Define an environment for cventry
\newenvironment{cventries}{
  \begin{center}
}{
  \end{center}
}
% Define an entry of cv information
% Usage: \cventry{<position>}{<title>}{<location>}{<date>}{<description>}
\newcommand*{\cventry}[5]{
  \setlength\tabcolsep{0pt}
  \setlength{\extrarowheight}{0pt}
  \begin{tabular*}{\textwidth}{@{\extracolsep{\fill}} L{12.5cm} R{4.5cm}}
    \ifempty{#2#3}
      {\entrypositionstyle{#1} & \entrydatestyle{#4} \\}
      {\entrytitlestyle{#2} & \entrylocationstyle{#3} \\
      \entrypositionstyle{#1} & \entrydatestyle{#4} \\}
    \multicolumn{2}{L{17cm}}{\descriptionstyle{#5}} \\
  \end{tabular*}
  \par
}

% Define an environment for cvskill
\newenvironment{cvskills}{
  \begin{center}
    \setlength\tabcolsep{1ex}
    \setlength{\extrarowheight}{0pt}
    \begin{tabular*}{\textwidth}{@{\extracolsep{\fill}} r L{15.2cm}}
}{
    \end{tabular*}
  \end{center}
}
% Define a line of cv information(skill)
% Usage: \cvskill{<type>}{<skillset>}
\newcommand*{\cvskill}[2]{
	\skilltypestyle{#1} & \skillsetstyle{#2}
  \\
}

% Define an environment for cvitems(for cventry)
\newenvironment{cvitems}{
  \vspace{-4mm}
  \begin{justify}
  \begin{itemize}[leftmargin=2ex, nosep, noitemsep]
    \setlength{\parskip}{0pt}
    \renewcommand{\labelitemi}{\bullet}
}{
  \end{itemize}
  \end{justify}
  \vspace{-4mm}
}
