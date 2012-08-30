#sbs-git:slp/pkgs/n/nfc-plugin-nxp nfc-plugin-nxp 0.0.1 373e1f7d458128eca88b9f4ea73d798394449e18
%define _optdir	/opt
%define _appdir	%{_optdir}/apps
%define _ugdir	%{_optdir}/ug


Name:       nfc-plugin-nxp
Summary:    NFC Plugin for NXP Solution
Version:    0.0.2
Release:    2
Group:      TO_BE/FILLED_IN
License:    TO BE FILLED IN
Source0:    %{name}-%{version}.tar.gz
BuildRequires: cmake
BuildRequires: pkgconfig(aul)
BuildRequires: pkgconfig(syspopup-caller)
BuildRequires: pkgconfig(glib-2.0)
BuildRequires: pkgconfig(ecore)
BuildRequires: pkgconfig(vconf)
BuildRequires: pkgconfig(elementary)
BuildRequires: pkgconfig(mm-common)
BuildRequires: pkgconfig(mm-sound)
BuildRequires: pkgconfig(security-server)
BuildRequires: pkgconfig(contacts-service)
BuildRequires: pkgconfig(bluetooth-api)
BuildRequires: pkgconfig(gconf-2.0)
BuildRequires: pkgconfig(dlog)
BuildRequires: pkgconfig(memo)
BuildRequires: pkgconfig(nfc-common-lib)

%description
Description: NFC Plugin for NXP Solution


%prep
%setup -q

%build
cmake . -DCMAKE_INSTALL_PREFIX=%{_prefix}

make %{?jobs:-j%jobs}


%install
rm -rf %{buildroot}
%make_install

%post 
# file owner
if [ ${USER} == "root" ]
then
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -g 6517 -i
else
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom ""
fi


%files
%defattr(-,root,root,-) 
/usr/lib/libnfc-plugin.so
